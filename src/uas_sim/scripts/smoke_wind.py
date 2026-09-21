#!/usr/bin/env python3
"""Actual one-GUI/two-PX4 smoke test, never starts navigation or commands takeoff.
Run from the workspace after sourcing ROS and the built overlay.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def main():
    root = Path.cwd()/'wind_runs'
    root.mkdir(exist_ok=True)
    (root/'smoke_result.json').write_text(json.dumps({'passed': False, 'status': 'running_or_incomplete'}))
    log = (root/'smoke_launch.log').open('w')
    proc = subprocess.Popen(['ros2','launch','uas_sim','wind_experiment.launch.py',
                             'start_navigation:=false','monitor_gui:=false'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('wind_smoke_observer')
    latest = {}
    samples = []
    node.create_subscription(String, '/aircells/state', lambda msg: latest.update(json.loads(msg.data)), 10)
    pub = node.create_publisher(Vector3, '/aircells/cell_L0_0008/wind_cmd', 10)
    start = time.monotonic(); onset = None; calm = None; peak = 20; result = None
    try:
        while time.monotonic()-start < 180:
            if proc.poll() is not None:
                raise RuntimeError(f'Launch exited {proc.returncode}; inspect smoke_launch.log')
            rclpy.spin_once(node, timeout_sec=0.1)
            if not latest or latest.get('observations_stale'): continue
            cells = {c['id']:c for c in latest['cells']}
            middle = cells['cell_L0_0008']
            t = latest['sim_time']
            if {v['name'] for v in latest['vehicles']} != {'x500_0','x500_1'}: continue
            if not all(c['panels_found'] for c in cells.values()): continue
            if onset is None:
                onset = t
                print(f'Both aircraft and all panels observed at simulation {t:.2f}s', flush=True)
            cross = 6 if t-onset < 12 else 0
            yaw = -0.997447349598712
            pub.publish(Vector3(x=-math.sin(yaw)*cross, y=math.cos(yaw)*cross, z=0.0))
            peak = max(peak, middle['applied_width'])
            samples.append(latest.copy())
            if cross == 0 and calm is None: calm = t
            if calm and t-calm > 28 and middle['applied_width'] < peak-3:
                assert peak > 29, peak
                assert all(abs(c['applied_width']-20)<0.1 for k,c in cells.items() if k != 'cell_L0_0008')
                assert all(c['width_deficit'] >= 0 for c in cells.values())
                result = dict(passed=True, peak_width=peak, final_width=middle['applied_width'], sim_time=t,
                              observed_vehicles=[v['name'] for v in latest['vehicles']], navigation=False)
                break
        if result is None: raise RuntimeError('Timed out waiting for wind expansion and contraction')
        (root/'smoke_result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    finally:
        (root/'smoke_states.json').write_text(json.dumps(samples))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL)
        log.close()


if __name__ == '__main__': main()
