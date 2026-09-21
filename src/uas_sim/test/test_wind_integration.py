"""Geometry and failure-path regression tests; these do not replace SITL tests."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from uas_sim.wind.prepare import prepare
from uas_sim.wind.cell_core import WidthPolicy
from uas_sim.wind.fit_envelope import fit

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT/'src/uas_sim/config/wind_demo.json'


def test_generation_preserves_sources_and_fixed_geometry(tmp_path):
    world, route = ROOT/'worlds/corridor.sdf', ROOT/'worlds/route.json'
    before = [x.read_bytes() for x in (world, route)]
    cfg = prepare(world, route, CONFIG, tmp_path)
    assert [x.read_bytes() for x in (world, route)] == before
    tree = ET.parse(cfg['world_file'])
    cells = cfg['cells']
    assert [x['index'] for x in cells] == [7, 8, 9]
    assert all(x['length'] == 180 and x['available_width'] == 50 for x in cells)
    plugin = tree.find(".//plugin[@name='aircells::AirCellWind']")
    assert plugin.get('filename') == 'libAirCellWind.so'
    for cell in cells:
        assert tree.find(f".//model[@name='{cell['id']}']") is not None
        for side in ('left', 'right'):
            panel = tree.find(f".//model[@name='{cell['id']}_side_{side}']")
            assert panel is not None and panel.findall('.//visual')
            assert not panel.findall('.//collision')
    changed = json.loads(CONFIG.read_text()); changed['initial_width_m'] = 40
    custom = tmp_path/'custom.json'; custom.write_text(json.dumps(changed))
    cfg2 = prepare(world, route, custom, tmp_path/'wide')
    plugin2 = ET.parse(cfg2['world_file']).find(".//plugin[@name='aircells::AirCellWind']")
    for a,b in zip(plugin.findall('cell'), plugin2.findall('cell')):
        for key in ('center','yaw','length','height','available_width'):
            assert a.findtext(key) == b.findtext(key)


@pytest.mark.parametrize('mutation', ['bend', 'adjacent_bend', 'overlap', 'altitude'])
def test_reject_unsupported_routes(tmp_path, mutation):
    route = json.loads((ROOT/'worlds/route.json').read_text())
    if mutation == 'bend': route['cells'][8]['x'] += 10
    if mutation == 'adjacent_bend': route['cells'][53]['x'] += 10
    if mutation == 'altitude': route['cells'][8]['z'] += 10
    if mutation == 'overlap':
        for c in route['cells']:
            if c['lane'] == 1:
                c['x'] += 10*0.8400890418867173
                c['y'] += 10*0.5424485244720069
    source = tmp_path/'route.json'; source.write_text(json.dumps(route))
    with pytest.raises(ValueError): prepare(ROOT/'worlds/corridor.sdf', source, CONFIG, tmp_path/'out')


def test_hysteresis_memory_and_pause():
    c = json.loads(CONFIG.read_text())['controller']
    p = WidthPolicy(c, 20.5)
    for i in range(300): assert p.update(i/10, 0).commanded_width == 20.5
    p = WidthPolicy(c, 40)
    p.update(0, 6)
    assert p.update(0, 0).commanded_width == 40
    for i in range(1, 20): assert p.update(i/10, 0).design_crosswind == 6
    assert p.update(2.1, 0).design_crosswind == 0


def test_fitter_rejects_evaluation_and_duplicates(tmp_path):
    paths=[]
    for i in range(3):
        run=tmp_path/str(i);run.mkdir()
        (run/'config.json').write_text(json.dumps({'run_role':'evaluation'}))
        path=run/'vehicles.csv';path.write_text('force_limited\nfalse\n');paths.append(path)
    with pytest.raises(ValueError, match='Evaluation'): fit(paths,1,0.5,1)
    for p in paths:
        (p.parent/'config.json').write_text(json.dumps({'run_role':'calibration'}))
        (p.parent/'observations.jsonl').write_text('{"vehicles":[]}\n')
    with pytest.raises(ValueError, match='Duplicate'): fit(paths,1,0.5,1)


@pytest.mark.parametrize('policy, expected', [('fixed_nominal', 20), ('fixed_worst', 50)])
def test_baseline_applied_from_initial_world(tmp_path, policy, expected):
    cfg = prepare(ROOT/'worlds/corridor.sdf', ROOT/'worlds/route.json', CONFIG, tmp_path, policy=policy)
    plugin = ET.parse(cfg['world_file']).find(".//plugin[@name='aircells::AirCellWind']")
    assert all(float(c.findtext('initial_width')) == expected for c in plugin.findall('cell'))
    assert cfg['initial_width_m'] == expected


def manager_harness():
    # Exercise the real callback without DDS. Transport is separately covered by SITL.
    import io
    from types import SimpleNamespace
    from uas_sim.wind.cell_manager import CellManager
    cfg=json.loads(CONFIG.read_text())
    cfg['cells']=[dict(id='cell',yaw=0,x=0,y=0,z=120,length=180,height=50,available_width=50)]
    cfg['initial_width_m']=40
    outputs=[]; commands=[]
    node=SimpleNamespace(cfg=cfg, cells={'cell':cfg['cells'][0]},
        policies={'cell':WidthPolicy(cfg['controller'],40)},
        commands={'cell':SimpleNamespace(publish=lambda msg:commands.append(msg.data))},
        state_pub=SimpleNamespace(publish=lambda msg:outputs.append(json.loads(msg.data))),
        raw_file=io.StringIO(),events_file=io.StringIO(),cell_file=io.StringIO(),vehicle_file=io.StringIO(),
        cell_log=SimpleNamespace(writerow=lambda row:None),vehicle_log=SimpleNamespace(writerow=lambda row:None),
        last_seen=None,last_t=None,latest=None,error='',reset_latched=False,previous_warning=0,
        get_logger=lambda:SimpleNamespace(error=lambda msg:None, warning=lambda msg:None))
    node.watchdog=lambda:CellManager.watchdog(node)
    node.observe=lambda obs:CellManager.on_observation(node,SimpleNamespace(data=json.dumps(obs)))
    return node, outputs, commands


def observation(t, missing=False, width=40):
    return dict(sim_time=t,cells=[dict(id='cell',wind=[0,0,0],width=width,panels_found=True,width_command_fresh=True)],
        vehicles=[dict(name=f'x500_{i}',position=[0,0,120],velocity=[0,0,0],wind=[0,0,0],force=[0,0,0],force_limited=False)
                  for i in range(1 if missing else 2)])


def test_manager_missing_aircraft_stale_and_reset_latch():
    node, states, commands=manager_harness()
    for i in range(150): node.observe(observation(i/10,missing=True))
    assert commands and all(w>=40 for w in commands)
    assert states[-1]['cells'][0]['reason']=='missing_aircraft'
    count=len(commands)
    node.last_seen=0; node.watchdog()
    assert states[-1]['observations_stale']
    assert not states[-1]['cells'][0]['observation_fresh']
    assert len(commands)==count
    node.observe(observation(0))
    assert node.reset_latched
    node.observe(observation(100))
    assert len(commands)==count


def test_manager_required_applied_deficit_and_pause():
    node, states, commands=manager_harness()
    obs=observation(0, width=20)
    obs['cells'][0]['wind']=[0,10,0]
    node.observe(obs)
    state=states[-1]['cells'][0]
    assert state['required_width']>50 and state['width_deficit']>50
    assert not state['admission_advisory']
    count=len(commands);node.observe(obs)
    assert len(commands)==count


def test_empirical_fit_monotone_domain_and_whole_run_cap(tmp_path):
    import csv
    paths=[]
    for i in range(3):
        run=tmp_path/str(i);run.mkdir()
        (run/'config.json').write_text(json.dumps({'run_role':'calibration'}))
        (run/'observations.jsonl').write_text('{"vehicles":[{"force_limited":false}]}\n')
        path=run/'vehicles.csv';paths.append(path)
        with path.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=['force_limited','in_altitude_band','wind_along_m_s','wind_up_m_s','wind_cross_m_s','cross_error_m','speed_m_s'])
            writer.writeheader()
            for wind in (0, 1, 2):
                writer.writerow(dict(force_limited=False,in_altitude_band=True,wind_along_m_s=0,wind_up_m_s=0,wind_cross_m_s=wind,cross_error_m=3-wind+i/10,speed_m_s=5+i))
    table=fit(paths,1,0.5,2)
    errors=[b['error_bound_m'] for b in table['bins']]
    assert errors==sorted(errors)
    assert table['crosswind_range_m_s']==[0,2]
    cfg=json.loads(CONFIG.read_text())['controller'];cfg['model']='table'
    assert WidthPolicy(cfg,20,table).envelope(2.1)[1] is False
    (paths[0].parent/'observations.jsonl').write_text('{"vehicles":[{"force_limited":true}]}\n')
    with pytest.raises(ValueError,match='saturation'): fit(paths,1,0.5,2)


def test_invalid_observation_closes_advice():
    node,states,commands=manager_harness()
    node.observe(observation(0))
    obs=observation(0.1);obs['cells'][0]['wind']=[float('nan'),0,0]
    count=len(commands);node.observe(obs)
    assert len(commands)==count
    assert states[-1]['observations_stale']
    assert not states[-1]['cells'][0]['admission_advisory']
