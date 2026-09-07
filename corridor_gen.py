#!/usr/bin/env python3
"""Generate the Denton -> Dallas air corridor: Gazebo world + route JSON.

ONE source of truth: the same route.json feeds both the cells placed in the
world and the waypoints the drones fly, so drones are always inside the cells.

Usage:
    python3 corridor_gen.py --miles 20 --lanes 1 --altitude 120
    python3 corridor_gen.py --miles 15 --lanes 2 --dest dfw
"""

import argparse
import json
import math

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


def vertiport_sdf(name, x, y, colour):
    """A landing pad plus a tall marker so it is findable from a distance."""
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name='pad'>
        <collision name='c'><geometry><cylinder>
          <radius>30</radius><length>2</length></cylinder></geometry></collision>
        <visual name='v'><geometry><cylinder>
          <radius>30</radius><length>2</length></cylinder></geometry>
          <material><ambient>0.15 0.15 0.18 1</ambient>
                    <diffuse>0.25 0.25 0.30 1</diffuse></material></visual>
      </link>
      <link name='ring'>
        <pose>0 0 1.2 0 0 0</pose>
        <visual name='v'><geometry><cylinder>
          <radius>18</radius><length>0.4</length></cylinder></geometry>
          <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse>
                    <emissive>{colour} 1</emissive></material></visual>
      </link>
      <link name='mast'>
        <pose>0 0 60 0 0 0</pose>
        <visual name='v'><geometry><cylinder>
          <radius>1.5</radius><length>120</length></cylinder></geometry>
          <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse>
                    <emissive>{colour} 1</emissive></material></visual>
      </link>
    </model>"""


def write_world(d, path):
    o = d['origin']
    ux, uy = d['unit_along']
    cam_x, cam_y = -3000 * ux, -3000 * uy      # behind the Denton end
    cam_yaw = math.pi / 2 - d['bearing_rad']   # look down the corridor
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
        <camera_pose>{cam_x:.1f} {cam_y:.1f} 2500 0 0.42 {cam_yaw:.4f}</camera_pose>
        <camera_clip><near>1.0</near><far>120000</far></camera_clip>
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

    sx, sy, sz = d['cell_size']
    # two shades per lane so individual cell boundaries stay visible
    palette = [(('0.10 0.45 0.95', '0.25 0.60 1.00')),
               (('0.95 0.40 0.10', '1.00 0.55 0.25')),
               (('0.15 0.70 0.30', '0.30 0.85 0.45'))]
    for c in d['cells']:
        a, b = palette[c['lane'] % len(palette)]
        rgb = a if c['index'] % 2 == 0 else b
        out.append(f"""
    <model name='{c['id']}'>
      <static>true</static>
      <pose>{c['x']:.2f} {c['y']:.2f} {c['z']:.2f} 0 0 {c['gz_yaw']:.6f}</pose>
      <link name='cell'>
        <visual name='vol'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <transparency>0.55</transparency>
          <material>
            <ambient>{rgb} 1</ambient>
            <diffuse>{rgb} 1</diffuse>
            <emissive>{rgb} 1</emissive>
          </material>
        </visual>
      </link>
    </model>""")

    # distance markers every 2 km, for scale
    ux2, uy2 = d['unit_along']
    step = 2000.0
    n_marks = int(d['segment_m'] // step)
    for k in range(1, n_marks + 1):
        mx, my = k * step * ux2, k * step * uy2
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
    with open(path, 'w') as f:
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
    a = p.parse_args()

    d = build(a.dest, a.miles if a.miles > 0 else None, a.lanes, a.altitude)
    write_world(d, a.world)
    with open(a.route, 'w') as f:
        json.dump(d, f, indent=2)

    print(f"Denton -> {a.dest}: {d['route_total_mi']:.1f} mi total, "
          f"bearing {d['bearing_deg']:.1f} deg")
    print(f"Built {d['segment_mi']:.1f} mi corridor: "
          f"{d['cells_per_lane']} cells x {a.lanes} lane(s) "
          f"= {len(d['cells'])} models, altitude {a.altitude} m")
    print(f"Wrote {a.world} and {a.route}")


if __name__ == '__main__':
    main()