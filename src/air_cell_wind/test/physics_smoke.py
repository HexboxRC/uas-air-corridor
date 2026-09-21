#!/usr/bin/env python3
"""Real Gazebo force fixture (simple inertial bodies, not a PX4 flight test).
Run only after stopping other Gazebo instances. No GUI or PX4 is started.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from gz.transport13 import Node
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.double_pb2 import Double


def main():
    folder = Path(tempfile.mkdtemp(prefix='aircell_physics_'))
    world = folder/'fixture.sdf'
    models = ''.join(f'''<model name="x500_{i}"><pose>0 {y} 120 0 0 0</pose>
      <link name="body"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      <collision name="body"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>'''
      for i,y in enumerate((15, 35)))
    world.write_text(f'''<sdf version="1.9"><world name="wind_fixture"><gravity>0 0 0</gravity>
    <physics name="physics" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="libAirCellWind.so" name="aircells::AirCellWind">
    <air_density>1.225</air_density><drag_area>0.08</drag_area><force_cap>5</force_cap>
    <wind_response>0.5</wind_response><edge_blend>10</edge_blend><command_timeout>1</command_timeout>
    <cell><id>test</id><center>0 0 120</center><yaw>0</yaw><length>180</length><height>50</height><available_width>50</available_width><initial_width>20</initial_width></cell>
    <vehicle>x500_0</vehicle><vehicle>x500_1</vehicle></plugin>{models}</world></sdf>''')
    node=Node(); observations=[]
    node.subscribe(StringMsg, '/aircells/observations', lambda msg: observations.append(json.loads(msg.data)))
    wind=node.advertise('/aircells/test/wind_cmd',Vector3d)
    width=node.advertise('/aircells/test/width_cmd',Double)
    log=(folder/'server.log').open('w')
    proc=subprocess.Popen(['gz','sim','-s','-r',str(world)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        start=time.monotonic()
        while time.monotonic()-start<35:
            if proc.poll() is not None: raise RuntimeError((folder/'server.log').read_text())
            if observations:
                t=observations[-1]['sim_time']
                if t<2: wind.publish(Vector3d())
                elif t<6: wind.publish(Vector3d(y=4))
                width.publish(Double(data=20 if t<4 else 40))
                if t>11: break
            time.sleep(0.05)
        assert observations and observations[-1]['sim_time']>11, 'No completed simulation'
        rows=[(o['sim_time'],v) for o in observations for v in o['vehicles']]
        assert all(len(v['orientation_xyzw']) == 4 and abs(sum(x*x for x in v['orientation_xyzw'])-1)<1e-6 for t,v in rows)
        calm=[v for t,v in rows if 0.5<t<1.8]
        assert calm and all(sum(abs(x) for x in v['force'])<1e-9 for v in calm)
        outside=[v for t,v in rows if v['name']=='x500_1']
        assert outside and all(sum(abs(x) for x in v['force'])<1e-9 for v in outside)
        assert all(sum(abs(x) for x in v['velocity'])<1e-9 for v in outside)
        inside=[v for t,v in rows if 3<t<3.9 and v['name']=='x500_0']
        assert inside and all(v['wind'][1]>3 and v['force'][1]>0 for v in inside)
        assert max(v['velocity'][1] for v in inside)>0.1
        assert all(v['position'][1]>10 for v in inside), 'Probe must lie outside initial operational width'
        wide=[v for t,v in rows if 4.1<t<4.8 and v['name']=='x500_0']
        assert wide and all(v['wind'][1]>3 for v in wide), 'Weather disappeared on resize'
        assert observations[-1]['cells'][0]['wind'][1]<0.01, 'Wind command timeout failed'
        assert not any(v['force_limited'] for t,v in rows)
        (folder/'observations.json').write_text(json.dumps(observations))
        print(f'PASS: calm zero force, localization, physical acceleration, wind outside operational width, width-independent weather, command timeout. Evidence: {folder}')
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
        log.close()

if __name__=='__main__': main()
