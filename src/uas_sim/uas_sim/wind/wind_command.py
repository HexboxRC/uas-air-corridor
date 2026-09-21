#!/usr/bin/env python3
"""Continuously command local wind. Ctrl+C sends calm; plugin also times out.

Wind coordinates are east/north/up; --crosswind is signed toward the left of
the corridor's direction of travel. This is a disturbance driver, not an input
to the cell sizing policy: the policy consumes plugin observations only.
"""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Vector3


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--runtime', default=str(Path.cwd()/'wind_runs/latest_runtime.json'))
    p.add_argument('--index', type=int, default=8)
    p.add_argument('--crosswind', type=float, required=True)
    p.add_argument('--alongwind', type=float, default=0.0)
    p.add_argument('--vertical', type=float, default=0.0)
    a = p.parse_args()
    cfg = json.loads(Path(a.runtime).read_text())
    cell = next((c for c in cfg['cells'] if c['index']==a.index), None)
    if cell is None:
        p.error('--index must be one of the configured study indices')
    if not all(math.isfinite(v) and abs(v)<=20 for v in (a.crosswind,a.alongwind,a.vertical)):
        p.error('Use finite wind components within +/-20 m/s for this experiment')
    c,s = math.cos(cell['yaw']), math.sin(cell['yaw'])
    wind = Vector3(x=c*a.alongwind-s*a.crosswind, y=s*a.alongwind+c*a.crosswind, z=a.vertical)
    rclpy.init(); node = rclpy.create_node('local_wind_driver')
    pub = node.create_publisher(Vector3, f"/aircells/{cell['id']}/wind_cmd", 1)
    timer = node.create_timer(0.1, lambda:pub.publish(wind))
    print(f"Wind at {cell['id']}: ENU=({wind.x:.3f},{wind.y:.3f},{wind.z:.3f}) m/s.")
    print('Keep running. Ctrl+C requests calm. Actual wind ramps with the plugin time constant.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        # If Ctrl+C already shut down ROS, the server's simulation-time timeout
        # will request calm. Do not attempt to publish on an invalid context.
        if rclpy.ok():
            for _ in range(3):
                pub.publish(Vector3()); rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
