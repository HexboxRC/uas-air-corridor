#!/usr/bin/env python3
"""Generate the Denton -> Dallas air corridor: Gazebo world + route JSON.

ONE source of truth: the same route.json feeds both the cells placed in the
world and the waypoints the drones fly, so waypoint centres and visible cell frames use the same coordinates.

Usage:
    python3 corridor_gen.py --miles 20 --lanes 1 --altitude 120
    python3 corridor_gen.py --miles 15 --lanes 2 --dest dfw
"""

import argparse
import json
import math
from pathlib import Path

DENTON = (33.2148, -97.1331)
DESTS = {
    'dallas': (32.7767, -96.7970),   # downtown, 36 mi, brg 147
    'dfw':    (32.8998, -97.0403),   # airport,  22 mi, brg 166
}

R_EARTH = 6371000.0
MI = 1609.34

# Air cell dimensions from the air cell paper (metres)
CELL_L, CELL_W, CELL_H = 200.0, 50.0, 50.0
CELL_SPACING = 180.0
LANE_SPACING = 50.0


def haversine(a, b):
    la1, lo1 = map(math.radians, a)
    la2, lo2 = map(math.radians, b)
    d1, d2 = la2 - la1, lo2 - lo1
    h = math.sin(d1/2)**2 + math.cos(la1)*math.cos(la2)*math.sin(d2/2)**2
    return 2 * R_EARTH * math.asin(math.sqrt(h))


def bearing(a, b):
    la1, lo1 = map(math.radians, a)
    la2, lo2 = map(math.radians, b)
    dl = lo2 - lo1
    y = math.sin(dl) * math.cos(la2)
    x = math.cos(la1)*math.sin(la2) - math.sin(la1)*math.cos(la2)*math.cos(dl)
    return math.atan2(y, x)


def build(dest_key, miles, lanes, altitude):
    dest = DESTS[dest_key]
    total_m = haversine(DENTON, dest)
    brg = bearing(DENTON, dest)

    span_m = min(miles * MI, total_m) if miles else total_m
    n_cells = max(int(math.ceil((span_m - CELL_L) / CELL_SPACING) + 1), 2)

    # Local ENU frame, origin at Denton: +x east, +y north
    ux, uy = math.sin(brg), math.cos(brg)      # along corridor
    px, py = math.cos(brg), -math.sin(brg)     # perpendicular (lane offset)

    cells = []
    for lane in range(lanes):
        off = (lane - (lanes - 1) / 2.0) * LANE_SPACING
        for i in range(n_cells):
            d = i * CELL_SPACING
            cells.append({
                'id': f'cell_L{lane}_{i:04d}',
                'lane': lane, 'index': i,
                'x': d * ux + off * px,
                'y': d * uy + off * py,
                'z': altitude,
                'yaw': brg,                    # compass bearing (for PX4)
                'gz_yaw': math.pi/2 - brg,     # Gazebo ENU yaw (for SDF)
            })

    return {
        'origin': {'lat': DENTON[0], 'lon': DENTON[1], 'alt': 200.0},
        'destination': dest_key,
        'route_total_m': total_m,
        'route_total_mi': total_m / MI,
        'segment_m': span_m,
        'segment_mi': span_m / MI,
        'bearing_rad': brg,
        'bearing_deg': math.degrees(brg) % 360.0,
        'unit_along': [ux, uy],
        'cell_size': [CELL_L, CELL_W, CELL_H],
        'cell_spacing_m': CELL_SPACING,
        'cells_per_lane': n_cells,
        'lanes': lanes,
        'altitude_m': altitude,
        'cells': cells,
    }


def box_visual(name, size, pose, rgb):
    """Visual-only box; corridor guides must never be physical obstacles."""
    return f"""<visual name='{name}'>
      <pose>{' '.join(str(v) for v in pose)} 0 0 0</pose>
      <geometry><box><size>{' '.join(str(v) for v in size)}</size></box></geometry>
      <cast_shadows>false</cast_shadows>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>
      <emissive>{rgb} 1</emissive></material></visual>"""


def cell_frame(c, size, rgb):
    length, width, height = size
    t = 0.6
    visuals = []
    for side in (-1, 1):
        for level in (-1, 1):
            visuals.append(box_visual(f'rail_{side}_{level}',
                (length, t, t), (0, side * width / 2, level * height / 2), rgb))
    for end in (-1, 1):
        for level in (-1, 1):
            visuals.append(box_visual(f'cross_{end}_{level}',
                (t, width, t), (end * length / 2, 0, level * height / 2), rgb))
        for side in (-1, 1):
            visuals.append(box_visual(f'post_{end}_{side}',
                (t, t, height), (end * length / 2, side * width / 2, 0), rgb))
    # Small floor dashes indicate travel direction without covering the drone.
    for j, offset in enumerate((-60, 0, 60)):
        visuals.append(box_visual(f'floor_dash_{j}',
            (18, 1.2, 0.2), (offset, 0, -height / 2), '0.95 0.95 0.95'))
    return f"""<model name='{c['id']}'>
      <static>true</static>
      <pose>{c['x']:.6f} {c['y']:.6f} {c['z']:.6f} 0 0 {c['gz_yaw']:.9f}</pose>
      <link name='frame'>{''.join(visuals)}</link></model>"""


def vertiport_sdf(name, x, y, colour):
    # Flat visual pad on the ground; the old 2 m thick pad intersected a drone
    # spawned at z=0.3. Ground remains the physical contact surface.
    return f"""<model name='{name}'>
      <static>true</static><pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
      <link name='pad'>
      {box_visual('base', (45, 45, 0.04), (0, 0, 0.03), '0.12 0.15 0.19')}
      {box_visual('stripe_a', (25, 2, 0.03), (0, 0, 0.07), colour)}
      {box_visual('stripe_b', (2, 25, 0.03), (0, 0, 0.07), colour)}
      </link></model>"""


def write_world(d, path, camera='close'):
    o = d['origin']
    ux, uy = d['unit_along']
    first = next(c for c in d['cells'] if c['lane'] == 0 and c['index'] == 0)
    if camera == 'overview':
        target_x = ux * d['segment_m'] / 2
        target_y = uy * d['segment_m'] / 2
        target_z = d['altitude_m']
        back = max(700.0, d['segment_m'] * 0.65)
        cam_x, cam_y = target_x - back * ux, target_y - back * uy
        cam_z = max(600.0, d['segment_m'] * 0.65)
    else:
        target_x, target_y, target_z = first['x'], first['y'], first['z'] / 2
        cam_x = target_x - 160 * ux - 100 * uy
        cam_y = target_y - 160 * uy + 100 * ux
        cam_z = first['z'] + 50
    cam_yaw = math.atan2(target_y - cam_y, target_x - cam_x)
    cam_pitch = math.atan2(cam_z - target_z,
                          math.hypot(target_x - cam_x, target_y - cam_y))
    out = [f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='corridor'>
    <physics name='4ms' type='ignored'>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
    <plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'/>
    <plugin filename='gz-sim-air-pressure-system' name='gz::sim::systems::AirPressure'/>
    <plugin filename='gz-sim-magnetometer-system' name='gz::sim::systems::Magnetometer'/>
    <plugin filename='gz-sim-navsat-system' name='gz::sim::systems::NavSat'/>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{o['lat']}</latitude_deg>
      <longitude_deg>{o['lon']}</longitude_deg>
      <elevation>{o['alt']}</elevation>
    </spherical_coordinates>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.6 0.75 0.95 1</background>
      <grid>true</grid>
    </scene>
    <gui fullscreen='0'>
      <plugin filename='MinimalScene' name='3D View'>
        <gz-gui><title>3D View</title><property key='showTitleBar' type='bool'>false</property></gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.6 0.6 0.6</ambient_light>
        <background_color>0.6 0.75 0.95</background_color>
        <camera_pose>{cam_x:.1f} {cam_y:.1f} {cam_z:.2f} 0 {cam_pitch:.6f} {cam_yaw:.6f}</camera_pose>
        <camera_clip><near>0.05</near><far>120000</far></camera_clip>
      </plugin>
      <plugin filename='GzSceneManager' name='Scene Manager'/>
      <plugin filename='InteractiveViewControl' name='Interactive view control'/>
      <plugin filename='CameraTracking' name='Camera Tracking'/>
      <plugin filename='EntityContextMenuPlugin' name='Entity context menu'/>
      <plugin filename='WorldControl' name='World control'>
        <gz-gui><anchors target='3D View'>
          <line own='left' target='left'/><line own='bottom' target='bottom'/>
        </anchors></gz-gui>
        <use_event>true</use_event><start_paused>false</start_paused>
      </plugin>
      <plugin filename='WorldStats' name='World stats'>
        <gz-gui><anchors target='3D View'>
          <line own='right' target='right'/><line own='bottom' target='bottom'/>
        </anchors></gz-gui>
        <sim_time>true</sim_time><real_time_factor>true</real_time_factor>
      </plugin>
      <plugin filename='EntityTree' name='Entity tree'/>
    </gui>
    <light name='sun' type='directional'>
      <pose>0 0 500 0 0 0</pose>
      <direction>-0.4 0.2 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>false</cast_shadows>
    </light>
    <model name='ground'>
      <static>true</static>
      <link name='link'>
        <collision name='c'><geometry><plane><normal>0 0 1</normal>
          <size>80000 80000</size></plane></geometry></collision>
        <visual name='v'><geometry><plane><normal>0 0 1</normal>
          <size>80000 80000</size></plane></geometry>
          <material><ambient>0.35 0.4 0.35 1</ambient>
                    <diffuse>0.45 0.5 0.45 1</diffuse></material></visual>
      </link>
    </model>"""]

    palette = [('0.08 0.75 0.95', '0.25 0.55 1.0'),
               ('1.0 0.55 0.12', '1.0 0.72 0.30'),
               ('0.20 0.90 0.50', '0.40 0.75 0.35')]
    for c in d['cells']:
        rgb = palette[c['lane'] % len(palette)][c['index'] % 2]
        out.append(cell_frame(c, d['cell_size'], rgb))
        # Ground projection provides scale when looking down from the corridor.
        length, width, _ = d['cell_size']
        out.append(f"""<model name='ground_{c['id']}'>
          <static>true</static>
          <pose>{c['x']:.6f} {c['y']:.6f} 0 0 0 {c['gz_yaw']:.9f}</pose>
          <link name='guides'>
          {box_visual('left', (length, 0.8, 0.03), (0, width/2, 0.035), rgb)}
          {box_visual('right', (length, 0.8, 0.03), (0, -width/2, 0.035), rgb)}
          </link></model>""")

    # distance markers every 2 km, for scale
    ux2, uy2 = d['unit_along']
    step = 2000.0
    n_marks = int(d['segment_m'] // step)
    for k in range(1, n_marks + 1):
        mx = k * step * ux2 - 90 * uy2
        my = k * step * uy2 + 90 * ux2
        out.append(f"""
    <model name='km_{k*2:03d}'>
      <static>true</static>
      <pose>{mx:.1f} {my:.1f} 100 0 0 0</pose>
      <link name='post'><visual name='v'>
        <geometry><cylinder><radius>4</radius><length>200</length></cylinder></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
                  <emissive>0.8 0.8 0.8 1</emissive></material>
      </visual></link>
    </model>""")

    # vertiports at both ends of the corridor
    first, last = d['cells'][0], d['cells'][d['cells_per_lane'] - 1]
    for tag, cell, col in (('vertiport_denton', first, '0.85 0.75 0.10'),
                           ('vertiport_dallas', last,  '0.85 0.75 0.10')):
        out.append(vertiport_sdf(tag, cell['x'], cell['y'], col))

    out.append("\n  </world>\n</sdf>\n")
    Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)
    with open(Path(path).expanduser(), 'w') as f:
        f.write(''.join(out))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dest', choices=list(DESTS), default='dallas')
    p.add_argument('--miles', type=float, default=20.0,
                   help='corridor length to build (0 = full route)')
    p.add_argument('--lanes', type=int, default=2)
    p.add_argument('--altitude', type=float, default=120.0)
    p.add_argument('--world', default='corridor.sdf')
    p.add_argument('--route', default='route.json')
    p.add_argument('--from-route', help='Use an existing route without changing its geometry')
    p.add_argument('--camera', choices=['close', 'overview'], default='close')
    a = p.parse_args()

    if a.from_route:
        with open(Path(a.from_route).expanduser()) as f:
            d = json.load(f)
    else:
        if a.lanes < 1 or a.altitude <= CELL_H / 2 or a.miles < 0:
            p.error('Use lanes >= 1, altitude > 25 m, and miles >= 0.')
        d = build(a.dest, a.miles if a.miles > 0 else None, a.lanes, a.altitude)
    write_world(d, a.world, camera=a.camera)
    # Preserve an existing route byte-for-byte when input and output coincide.
    route_path = Path(a.route).expanduser()
    same = a.from_route and Path(a.from_route).expanduser().resolve() == route_path.resolve()
    if not same:
        route_path.parent.mkdir(parents=True, exist_ok=True)
        with open(route_path, 'w') as f:
            json.dump(d, f, indent=2)
    print(f"{d['segment_mi']:.1f} mi, {d['lanes']} lanes, "
          f"{d['cells_per_lane']} cells/lane, altitude {d['altitude_m']} m")
    print(f'Wrote open corridor frames to {a.world}; camera={a.camera}')
    print(f'Route: {a.route}')



if __name__ == '__main__':
    main()