#!/usr/bin/env python3
"""Fit a monotone empirical cross-track error envelope from independent runs.

This produces a calibration table, not a certified probabilistic bound. Use
separate runs for validation. All bins need observations; missing bins fail.
"""
import argparse
import csv
import json
import math
import hashlib
from pathlib import Path


def fit(paths, step, extra, minimum):
    if step <= 0 or extra < 0 or minimum < 1:
        raise ValueError('Invalid fitting options')
    if len({Path(p).resolve() for p in paths}) < 3:
        raise ValueError('At least three independent calibration runs required')
    hashes = set()
    for path in paths:
        path = Path(path)
        cfg = json.loads((path.parent/'config.json').read_text())
        if cfg.get('run_role') != 'calibration':
            raise ValueError('Evaluation data cannot be used for calibration')
        fingerprint = hashlib.sha256(path.read_bytes()).hexdigest()
        if fingerprint in hashes:
            raise ValueError('Duplicate calibration data')
        hashes.add(fingerprint)
        raw = path.parent/'observations.jsonl'
        if not raw.exists():
            raise ValueError('Raw observations required to check whole-run force saturation')
        for line in raw.read_text().splitlines():
            if any(v['force_limited'] for v in json.loads(line)['vehicles']):
                raise ValueError('Force saturation occurred; reject whole run')
    bins = {}; speeds = []; winds = []; max_wind = 0.0; count = 0
    for path in paths:
        with Path(path).open() as f:
            rows=list(csv.DictReader(f))
            if any(row['force_limited'].lower()=='true' for row in rows):
                raise ValueError(f'{path}: force saturation occurred; exclude the whole run from calibration')
            for row in rows:
                if row['in_altitude_band'].lower()!='true':
                    continue
                if abs(float(row['wind_along_m_s'])) > 0.5 or abs(float(row['wind_up_m_s'])) > 0.1:
                    continue  # This first table calibrates crosswind-only experiments.
                w = abs(float(row['wind_cross_m_s'])); e = abs(float(row['cross_error_m']))
                speed = float(row['speed_m_s'])
                if not all(math.isfinite(x) for x in (w,e,speed)): continue
                index = max(0, math.ceil(w/step-1e-9)-1)
                bins.setdefault(index, []).append(e)
                speeds.append(speed); winds.append(w); max_wind = max(max_wind,w); count += 1
    if not bins or max_wind < step:
        raise ValueError('Not enough wind variation; collect calm and multiple wind levels')
    bound = 0.0; table = []
    for i in range(max(bins)+1):
        values = bins.get(i, [])
        if len(values) < minimum:
            raise ValueError(f'Wind bin {i*step:g}–{(i+1)*step:g} m/s has only {len(values)} samples')
        bound = max(bound, max(values)+extra)
        table.append(dict(upper_crosswind_m_s=min((i+1)*step, max_wind),
                          error_bound_m=bound, samples=len(values)))
    return dict(crosswind_range_m_s=[min(winds), max(winds)], bins=table, speed_range_m_s=[min(speeds),max(speeds)],
                alongwind_limit_m_s=0.5, vertical_wind_limit_m_s=0.1,
                calibration_files=[str(Path(p).resolve()) for p in paths], total_samples=count,
                added_empirical_margin_m=extra,
                warning='Empirical maxima plus a user margin, NOT a statistical or physical guarantee. Temporal samples are correlated. Validate on independent runs. Bounds depend on gust profile, speed, controller, aircraft, and force model.')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('logs', nargs='+', help='Independent calibration vehicles.csv files')
    p.add_argument('--bin-width', type=float, default=1.0)
    p.add_argument('--extra-margin', type=float, default=0.5)
    p.add_argument('--min-samples', type=int, default=30)
    p.add_argument('--output', default='envelope.json')
    a=p.parse_args()
    if a.bin_width<=0 or a.extra_margin<0 or a.min_samples<1:
        p.error('Invalid fitting options')
    if len(set(map(lambda x:str(Path(x).resolve()),a.logs))) < 3:
        p.error('Provide at least three independent calibration run files')
    result=fit(a.logs,a.bin_width,a.extra_margin,a.min_samples)
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    print(f'Wrote empirical envelope to {a.output}; validate with different run files.')


if __name__ == "__main__":
    main()
