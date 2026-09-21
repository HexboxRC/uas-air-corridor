#!/usr/bin/env python3
"""Create an experimental world copy and runtime config; preserve user inputs."""
import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def child(parent, name, value):
    node = ET.SubElement(parent, name)
    node.text = str(value)
    return node


def side_panel(cell, side, width):
    model = ET.Element('model', name=cell['id'] + ('_side_left' if side == 1 else '_side_right'))
    child(model, 'static', 'true')
    x = cell['x'] - math.sin(cell['yaw']) * side * width / 2
    y = cell['y'] + math.cos(cell['yaw']) * side * width / 2
    child(model, 'pose', f"{x} {y} {cell['z']} 0 0 {cell['yaw']}")
    link = ET.SubElement(model, 'link', name='frame')
    length, height, t = cell['length'], cell['height'], 0.65
    pieces = [(0, 0, h * height / 2, length, t, t) for h in (-1, 1)]
    pieces += [(end * length / 2, 0, 0, t, t, height) for end in (-1, 1)]
    for i, (x, y, z, a, b, c) in enumerate(pieces):
        visual = ET.SubElement(link, 'visual', name=f'edge_{i}')
        child(visual, 'pose', f'{x} {y} {z} 0 0 0')
        child(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size', f'{a} {b} {c}')
        child(visual, 'cast_shadows', 'false')
        mat = ET.SubElement(visual, 'material')
        for key in ('ambient', 'diffuse', 'emissive'):
            child(mat, key, '1 0.3 0.1 1')
    return model


def prepare(world_path, route_path, config_path, output_dir):
    cfg = json.loads(Path(config_path).read_text())
    route = json.loads(Path(route_path).read_text())
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('Expected a named SDF world')
    if any('WindEffects' in p.get('name', '') or 'AirCellWind' in p.get('name', '')
           for p in world.findall('plugin')):
        raise ValueError('Input already has a wind plugin; use the original corridor.sdf')
    if route.get('lanes', 0) < 1 or len(route['cells']) < 2:
        raise ValueError('Route must contain at least two cells')
    c = cfg['controller']
    if not 0 < c['min_width_m'] <= cfg['initial_width_m'] <= c['max_width_m'] <= cfg['available_width_m']:
        raise ValueError('Require 0 < minimum <= initial <= maximum <= available width')
    if any(not math.isfinite(v) or v <= 0 for v in cfg['physics'].values()):
        raise ValueError('Physics parameters must be positive finite numbers')
    from cell_core import WidthPolicy
    table = None
    if c['model'] == 'table':
        table_path = Path(c['envelope_file']).expanduser()
        if not table_path.is_absolute():
            table_path = Path(config_path).resolve().parent / table_path
        table = json.loads(table_path.read_text())
    WidthPolicy(c, cfg['initial_width_m'], table)
    lane = sorted([x for x in route['cells'] if x['lane'] == cfg['lane']], key=lambda x:x['index'])
    lane0 = sorted([x for x in route['cells'] if x['lane'] == 0], key=lambda x:x['index'])
    if cfg['lane'] != 0:
        raise ValueError('Existing navigation uses lane 0; this first experiment requires lane=0')
    if cfg['drones'] not in (1, 2) or cfg['spawn_stagger'] < 1:
        raise ValueError('Use one or two drones and a positive stagger for this prototype')
    if (cfg['drones'] - 1) * cfg['spawn_stagger'] >= len(lane0):
        raise ValueError('Not enough route cells for the requested spawns')
    heading = math.atan2(lane[-1]['y']-lane[0]['y'], lane[-1]['x']-lane[0]['x'])
    spacing = float(route['cell_spacing_m'])
    for a, b in zip(lane, lane[1:]):
        if abs(math.hypot(b['x']-a['x'], b['y']-a['y'])-spacing) > 0.1:
            raise ValueError('Prototype requires regularly spaced straight cells')
        if abs(math.atan2(math.sin(math.atan2(b['y']-a['y'], b['x']-a['x'])-heading),
                          math.cos(math.atan2(b['y']-a['y'], b['x']-a['x'])-heading))) > 1e-5:
            raise ValueError('Prototype requires a straight corridor')
    selected = []
    for index in cfg['study_indices']:
        source = next((x for x in lane if x['index'] == index), None)
        if source is None:
            raise ValueError(f'No cell {index} in lane {cfg["lane"]}')
        if not re.fullmatch(r'[A-Za-z0-9_]+', source['id']):
            raise ValueError('Cell identifiers must be alphanumeric with underscores')
        for other in route['cells']:
            if other['lane'] != cfg['lane'] and other['index'] == index:
                lane_distance = math.hypot(other['x']-source['x'], other['y']-source['y'])
                if cfg['available_width_m']/2 + route['cell_size'][1]/2 > lane_distance + 1e-5:
                    raise ValueError('Available width would overlap a neighboring nominal lane')
        cell = {k:source[k] for k in ('id', 'index', 'lane', 'x', 'y', 'z')}
        cell.update(yaw=heading, length=spacing, height=float(route['cell_size'][2]),
                    available_width=cfg['available_width_m'])
        selected.append(cell)
    if len({x['id'] for x in selected}) != len(selected):
        raise ValueError('Duplicate study indices')
    if not selected:
        raise ValueError('Select at least one study cell')
    # Do not remove user geometry unrelated to these exact study cells.
    for cell in selected:
        for model in list(world.findall('model')):
            if model.get('name') == cell['id']:
                world.remove(model)
        for side in (-1, 1):
            world.append(side_panel(cell, side, cfg['initial_width_m']))
    plugin = ET.SubElement(world, 'plugin', filename=str(ROOT/'build'/'libAirCellWind.so'),
                           name='aircells::AirCellWind')
    mappings = {'air_density':'air_density_kg_m3', 'drag_area':'drag_area_m2',
                'force_cap':'force_cap_n', 'wind_response':'wind_response_s',
                'edge_blend':'edge_blend_m', 'command_timeout':'wind_command_timeout_s'}
    for sdf_key, cfg_key in mappings.items():
        child(plugin, sdf_key, cfg['physics'][cfg_key])
    for cell in selected:
        e = ET.SubElement(plugin, 'cell')
        child(e, 'id', cell['id'])
        child(e, 'center', f"{cell['x']} {cell['y']} {cell['z']}")
        for key in ('yaw', 'length', 'height', 'available_width'):
            child(e, key, cell[key])
        child(e, 'initial_width', cfg['initial_width_m'])
    for i in range(cfg['drones']):
        child(plugin, 'vehicle', f'x500_{i}')
    # Preserve and explicitly ensure the camera plugin in the experimental world.
    gui = world.find('gui')
    if gui is not None and not any(p.get('filename') == 'CameraTracking' for p in gui.findall('plugin')):
        ET.SubElement(gui, 'plugin', filename='CameraTracking', name='Camera Tracking')
    out = Path(output_dir).resolve(); out.mkdir(parents=True, exist_ok=True)
    output_world = out/'corridor_wind.sdf'
    if output_world == Path(world_path).resolve():
        raise ValueError('Output cannot overwrite input world')
    tree.write(output_world, encoding='utf-8', xml_declaration=True)
    cfg.update(cells=selected, route_file=str(Path(route_path).resolve()),
               source_world=str(Path(world_path).resolve()), world_file=str(output_world),
               envelope_table=table, config_source=str(Path(config_path).resolve()))
    (out/'runtime.json').write_text(json.dumps(cfg, indent=2)+'\n')
    print(f'Created {output_world}\nStudy cells: ' + ', '.join(x['id'] for x in selected))
    print('Original world and route preserved. Coefficients are '+c['model'].upper()+'.')
    return cfg


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--world', default=str(Path.home()/'uas_ros2_ws/worlds/corridor.sdf'))
    p.add_argument('--route', default=str(Path.home()/'uas_ros2_ws/worlds/route.json'))
    p.add_argument('--config', default=str(ROOT/'config.json'))
    p.add_argument('--output', default=str(ROOT/'run'))
    a = p.parse_args()
    prepare(a.world, a.route, a.config, a.output)
