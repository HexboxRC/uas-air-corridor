#!/usr/bin/env python3
"""Summarize measured containment; no traffic-capacity claim is computed."""
import argparse
import csv
import json
from pathlib import Path


def summarize(run):
    run=Path(run)
    with (run/'cells.csv').open() as f: cells=list(csv.DictReader(f))
    with (run/'vehicles.csv').open() as f: vehicles=list(csv.DictReader(f))
    cruise=[x for x in vehicles if x['in_altitude_band'].lower()=='true']
    unsafe=[x for x in cruise if float(x['body_clearance_m'])<0]
    results={'run':str(run), 'cell_samples':len(cells), 'cruise_vehicle_samples':len(cruise),
             'sampled_body_containment_violations':len(unsafe),
             'min_body_clearance_m':min((float(x['body_clearance_m']) for x in cruise),default=None),
             'max_required_minus_applied_m':max((float(x['required_minus_applied']) for x in cells),default=None),
             'force_limited_samples':sum(x['force_limited'].lower()=='true' for x in vehicles),
             'admission_enforced':False,
             'warning':'Sampled results only; no between-sample guarantee. No throughput metric: admission is not enforced in this geometry/calibration prototype.'}
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',help='Directory containing cells.csv and vehicles.csv')
    print(json.dumps(summarize(p.parse_args().run),indent=2))
