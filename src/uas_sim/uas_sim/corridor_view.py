#!/usr/bin/env python3
"""Live, read-only corridor map. No flight commands are published.

The launch file passes the same route, spawn stagger and namespaces as navigation.
ENU reconstruction assumes each PX4 local origin stays at its configured spawn.
The --preview option is explicitly illustrative and never connects to ROS.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

BG = '#0b1220'
PANEL = '#111e30'
TEXT = '#eef4fa'
MUTED = '#91a4ba'
COLORS = ['#50e3cf', '#ffc46b', '#ce9eff', '#ff839b']
LANES = ['#277299', '#9b6933', '#477d50']


def clip_polygon(points, bounds):
    """Clip map geometry so a close view cannot draw over another panel."""
    x0, y0, x1, y1 = bounds
    for axis, edge, greater in [(0, x0, True), (0, x1, False),
                                (1, y0, True), (1, y1, False)]:
        out = []
        if not points:
            break
        prev = points[-1]
        pin = prev[axis] >= edge if greater else prev[axis] <= edge
        for cur in points:
            cin = cur[axis] >= edge if greater else cur[axis] <= edge
            if cin != pin:
                t = (edge - prev[axis]) / (cur[axis] - prev[axis])
                out.append((prev[0] + t * (cur[0] - prev[0]),
                            prev[1] + t * (cur[1] - prev[1])))
            if cin:
                out.append(cur)
            prev, pin = cur, cin
        points = out
    return points


class Drone:
    def __init__(self, index, spawn):
        self.index = index
        self.ns = f'px4_{index}'
        self.spawn = (spawn['x'], spawn['y'])
        self.xyz = None
        self.valid = False
        self.seen = 0.0
        self.status_seen = 0.0
        self.status = None
        self.yaw = float(spawn['gz_yaw'])
        self.trail = deque(maxlen=10000)

    def on_position(self, msg):
        self.valid = bool(msg.xy_valid and msg.z_valid
                          and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)))
        self.seen = time.monotonic()
        if not self.valid:
            return
        self.xyz = (self.spawn[0] + msg.y, self.spawn[1] + msg.x, -msg.z)
        heading = getattr(msg, 'heading', math.nan)
        if math.isfinite(heading):
            self.yaw = math.pi / 2 - heading
        if not self.trail or math.dist(self.xyz[:2], self.trail[-1]) >= 1.0:
            self.trail.append(self.xyz[:2])

    def on_status(self, msg):
        self.status = (int(msg.arming_state), int(msg.nav_state))
        self.status_seen = time.monotonic()

    def fresh(self):
        return self.valid and self.xyz is not None and time.monotonic() - self.seen < 3.0


def label(canvas, x, y, text, size=11, color=TEXT, anchor='nw', bold=False):
    canvas.create_text(x, y, text=text, fill=color, anchor=anchor,
                       font=('DejaVu Sans', size, 'bold' if bold else 'normal'))


def map_panel(canvas, rect, title, cells, size, drones, bounds, selected=None, route_axis=None):
    x, y, w, h = rect
    canvas.create_rectangle(x, y, x+w, y+h, fill=PANEL, outline='#26374b')
    label(canvas, x+15, y+12, title, 11, bold=True)
    viewport = (x+18, y+43, x+w-18, y+h-24)
    xmin, ymin, xmax, ymax = bounds
    if route_axis:
        ux, uy = route_axis
        ss = [c['x']*ux+c['y']*uy for c in cells]
        ll = [-c['x']*uy+c['y']*ux for c in cells]
        xmin, xmax = min(ss)-size[0]/2-100, max(ss)+size[0]/2+100
        ymin, ymax = min(ll)-size[1]/2-20, max(ll)+size[1]/2+20
        scale = (w-36)/(xmax-xmin)
        yscale = (h-67)/(ymax-ymin)
    else:
        scale = min((w-36)/max(xmax-xmin, 1), (h-67)/max(ymax-ymin, 1))
        yscale = scale
    cx, cy = x+w/2, (viewport[1]+viewport[3])/2
    mx, my = (xmin+xmax)/2, (ymin+ymax)/2

    def project(east, north):
        if route_axis:
            east, north = east*ux+north*uy, -east*uy+north*ux
        return cx+(east-mx)*scale, cy-(north-my)*yscale

    def inside(point, margin=0):
        return (viewport[0]+margin <= point[0] <= viewport[2]-margin
                and viewport[1]+margin <= point[1] <= viewport[3]-margin)

    length, width, _ = size
    for c in cells:
        yaw = c['gz_yaw']
        co, si = math.cos(yaw), math.sin(yaw)
        corners = [project(c['x']+a*co-b*si, c['y']+a*si+b*co)
                   for a, b in [(-length/2, -width/2), (length/2, -width/2),
                                (length/2, width/2), (-length/2, width/2)]]
        corners = clip_polygon(corners, viewport)
        if len(corners) >= 3:
            canvas.create_polygon(*[v for p in corners for v in p],
                                  fill='#172839', outline=LANES[c['lane'] % len(LANES)])
        center = project(c['x'], c['y'])
        if selected is not None and inside(center, 30):
            label(canvas, center[0], center[1]+12,
                  f"L{c['lane']} / {c['index']}", 8, MUTED, 'n')

    for drone in drones:
        color = COLORS[drone.index % len(COLORS)]
        prev = None
        # Limit drawing work on long flights, preserving the full stored trail.
        trail = list(drone.trail)
        stride = max(1, len(trail)//1500)
        for point in trail[::stride]:
            cur = project(*point)
            if prev is not None and inside(prev) and inside(cur):
                canvas.create_line(*prev, *cur, fill=color, width=2)
            prev = cur
        pos = drone.xyz[:2] if drone.xyz else drone.spawn
        px, py = project(*pos)
        if not inside((px, py), 15):
            continue
        if not drone.xyz:
            canvas.create_oval(px-6, py-6, px+6, py+6, outline=color, width=2)
            text = f'D{drone.index} waiting'
        else:
            color = color if drone.fresh() else MUTED
            head = project(pos[0]+math.cos(drone.yaw), pos[1]+math.sin(drone.yaw))
            yaw = math.atan2(py-head[1], head[0]-px)
            arrow = [(px+11*math.cos(yaw), py-11*math.sin(yaw))]
            for a in (yaw+2.5, yaw-2.5):
                arrow.append((px+9*math.cos(a), py-9*math.sin(a)))
            canvas.create_polygon(*[v for p in arrow for v in p], fill=color, outline=BG)
            text = f'D{drone.index}' + ('' if drone.fresh() else ' stale')
        # Draw labels inward at edges instead of spilling out of the panel.
        tx = min(max(px+15, viewport[0]+8), viewport[2]-90)
        ty = min(max(py-24, viewport[1]+4), viewport[3]-22)
        label(canvas, tx, ty, text, 10, color, bold=True)

    label(canvas, x+w-20, y+14,
          '→ along route · lateral scale enlarged' if route_axis else 'N ↑   E →',
          9, MUTED, 'ne')
    scale_m = 100 if selected is not None else 1000
    if scale_m*scale < w-70:
        bx, by = x+22, y+h-14
        canvas.create_line(bx, by, bx+scale_m*scale, by, fill=MUTED, width=2)
        label(canvas, bx+scale_m*scale+8, by, f'{scale_m:g} m', 8, MUTED, 'w')


def render(canvas, width, height, data, drones, preview=False, local_span=500):
    canvas.delete('all')
    canvas.create_rectangle(0, 0, width, height, fill=BG, outline=BG)
    label(canvas, 24, 15, 'AIR CORRIDOR  /  FLIGHT VIEW', 18, bold=True)
    mode = 'ILLUSTRATIVE PREVIEW — sample positions, not live telemetry' if preview else 'LIVE ROS 2 TELEMETRY'
    label(canvas, 24, 47, mode, 10, '#ffc46b' if preview else '#50e3cf')
    label(canvas, width-24, 20,
          f"{data['segment_mi']:g} mi  ·  {data['lanes']} lanes  ·  {data['altitude_m']:g} m altitude",
          10, MUTED, 'ne')
    card_w = (width-48-14*(len(drones)-1))/len(drones)
    ux, uy = data['unit_along']
    for i, drone in enumerate(drones):
        x = 24+i*(card_w+14)
        canvas.create_rectangle(x, 76, x+card_w, 164, fill=PANEL, outline='#26374b')
        label(canvas, x+14, 87, f'DRONE {i}  /  {drone.ns}', 11, COLORS[i % len(COLORS)], bold=True)
        if drone.xyz:
            along = drone.xyz[0]*ux+drone.xyz[1]*uy
            progress = min(100, max(0, along / max((data['cells_per_lane']-1)*data['cell_spacing_m'], 1)*100))
            text = f'{along/1000:.2f} km along route   ·   {drone.xyz[2]:.1f} m altitude   ·   {progress:.1f}%'
        else:
            text = 'Waiting for position — outlined map marker shows expected spawn'
        label(canvas, x+14, 113, text, 9)
        if not drone.fresh():
            state = 'NO FRESH VALID POSITION'
        elif drone.status is None or time.monotonic()-drone.status_seen > 3:
            state = 'Position live; status unavailable'
        else:
            armed, nav = drone.status
            state = f'{"ARMED" if armed == 2 else "DISARMED"}   ·   {"OFFBOARD" if nav == 14 else "nav state " + str(nav)}'
        label(canvas, x+14, 139, state, 9, MUTED)

    cells = data['cells']
    margin = max(data['cell_size'])
    bounds = (min(c['x'] for c in cells)-margin, min(c['y'] for c in cells)-margin,
              max(c['x'] for c in cells)+margin, max(c['y'] for c in cells)+margin)
    over_h = max(210, int((height-220)*0.45))
    map_panel(canvas, (24, 182, width-48, over_h), 'FULL ROUTE · both drone trails',
              cells, data['cell_size'], drones, bounds, route_axis=data['unit_along'])
    local_y = 196+over_h
    local_h = max(200, height-local_y-40)
    for i, drone in enumerate(drones):
        center = drone.xyz[:2] if drone.xyz else drone.spawn
        half = local_span/2
        bounds = (center[0]-half, center[1]-half, center[0]+half, center[1]+half)
        map_panel(canvas, (24+i*(card_w+14), local_y, card_w, local_h),
                  f'DRONE {i} · tracking close view', cells, data['cell_size'],
                  drones, bounds, selected=i)
    label(canvas, 24, height-24,
          'Drone symbols are enlarged. Close views use ENU coordinates; altitude is shown above. Both drones use lane 0.',
          9, MUTED)


def make_drones(data, count, stagger):
    lane = sorted((c for c in data['cells'] if c['lane'] == 0), key=lambda c: c['index'])
    if not 1 <= count <= 4 or stagger < 1 or (count-1)*stagger >= len(lane):
        raise ValueError('Use 1–4 drones with valid staggered spawn indices.')
    return [Drone(i, lane[i*stagger]) for i in range(count)]


def preview_positions(data, drones):
    """Illustrative snapshot for the preview only, never used by the live path."""
    ux, uy = data['unit_along']
    for drone in drones:
        travel = min(850.0, data['segment_m'] / 4)
        drone.xyz = (drone.spawn[0]+travel*ux, drone.spawn[1]+travel*uy,
                     float(data['altitude_m']))
        drone.valid = True
        drone.seen = drone.status_seen = time.monotonic()
        drone.status = (2, 14)
        drone.trail = deque([(drone.spawn[0]+travel*j/50*ux,
                              drone.spawn[1]+travel*j/50*uy) for j in range(51)],
                            maxlen=10000)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default=str(Path.home()/'uas_ros2_ws/worlds/route.json'))
    parser.add_argument('--drones', type=int, default=2)
    parser.add_argument('--stagger', type=int, default=4)
    parser.add_argument('--position-topic', default='vehicle_local_position_v1')
    parser.add_argument('--status-topic', default='vehicle_status_v4')
    parser.add_argument('--preview', action='store_true', help='Clearly labeled illustrative positions, no ROS connection')
    args = parser.parse_args()
    data = json.loads(Path(args.route).expanduser().read_text())
    drones = make_drones(data, args.drones, args.stagger)
    try:
        import tkinter as tk
    except ImportError:
        raise SystemExit('Install the viewer dependency: sudo apt install python3-tk')
    root = tk.Tk()
    root.title('UNT Air Corridor — Live Flight View' if not args.preview else 'Air Corridor — Illustrative Preview')
    root.geometry('1240x880')
    root.minsize(1050, 780)
    canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
    canvas.pack(fill='both', expand=True)
    node = None
    ros = None
    if not args.preview:
        # Match the working configuration established for this project.
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        os.environ['ROS_DOMAIN_ID'] = '0'
        os.environ['ROS_LOCALHOST_ONLY'] = '0'
        import rclpy as ros
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
        ros.init(args=[])
        node = ros.create_node(f'corridor_view_{os.getpid()}')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for drone in drones:
            node.create_subscription(VehicleLocalPosition,
                f'/{drone.ns}/fmu/out/{args.position_topic}', drone.on_position, qos)
            node.create_subscription(VehicleStatus,
                f'/{drone.ns}/fmu/out/{args.status_topic}', drone.on_status, qos)

    def update():
        if node is not None:
            for _ in range(50):
                ros.spin_once(node, timeout_sec=0.0)
        elif args.preview:
            preview_positions(data, drones)
        render(canvas, canvas.winfo_width(), canvas.winfo_height(), data, drones, args.preview)
        root.after(200, update)

    try:
        root.after(100, update)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
            if ros.ok():
                ros.shutdown()


if __name__ == '__main__':
    main()
