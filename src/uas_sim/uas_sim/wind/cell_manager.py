#!/usr/bin/env python3
"""Resize study-cell side panels, display current state and log ground truth.

Publishes geometry commands and ADVISORY entry flags. It does not command PX4
or enforce traffic admission. See IMPLEMENTATION_GUIDE.md before interpreting
the output as a safety or capacity result.
"""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String
from .cell_core import WidthPolicy, crosswind, local_coordinates, occupies_station


class CellManager(Node):
    def __init__(self, runtime, policy=None):
        super().__init__('air_cell_manager')
        self.cfg = json.loads(Path(runtime).read_text())
        if policy:
            self.cfg['controller']['policy'] = policy
        self.cells = {c['id']:c for c in self.cfg['cells']}
        self.policies = {key:WidthPolicy(self.cfg['controller'], self.cfg['initial_width_m'],
                                        self.cfg.get('envelope_table')) for key in self.cells}
        self.commands = {key:self.create_publisher(Float64, f'/aircells/{key}/width_cmd', 1)
                         for key in self.cells}
        self.state_pub = self.create_publisher(String, '/aircells/state', 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, '/aircells/observations', self.on_observation, qos)
        self.reset_latched = False
        self.last_seen = None
        self.last_t = None
        self.latest = None
        self.error = ''
        self.previous_warning = 0.0
        root = Path(self.cfg['log_root'])
        root.mkdir(exist_ok=True)
        label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.run_dir = root/label; self.run_dir.mkdir()
        (self.run_dir/'config.json').write_text(json.dumps(self.cfg, indent=2))
        self.cell_file = (self.run_dir/'cells.csv').open('w', newline='')
        self.vehicle_file = (self.run_dir/'vehicles.csv').open('w', newline='')
        self.cell_log = csv.DictWriter(self.cell_file, fieldnames=[
            'sim_time', 'cell_id', 'policy', 'model', 'crosswind', 'design_crosswind',
            'error_envelope', 'required_width', 'commanded_width', 'applied_width',
            'required_minus_applied', 'width_deficit', 'model_domain_valid', 'required_exceeds_available', 'observation_fresh', 'vehicles_complete', 'wind_command_fresh', 'occupants', 'admission_advisory', 'reason', 'panels_found'])
        self.vehicle_log = csv.DictWriter(self.vehicle_file, fieldnames=[
            'sim_time', 'vehicle', 'cell_id', 'along_m', 'cross_error_m', 'vertical_error_m',
            'speed_m_s', 'wind_cross_m_s', 'wind_along_m_s', 'wind_up_m_s', 'force_n',
            'force_limited', 'applied_width', 'body_clearance_m', 'model_clearance_m',
            'in_altitude_band', 'model', 'position_x', 'position_y', 'position_z',
            'velocity_x', 'velocity_y', 'velocity_z', 'wind_x', 'wind_y', 'wind_z',
            'force_x', 'force_y', 'force_z'])
        self.events_file = (self.run_dir/'events.jsonl').open('w')
        self.raw_file = (self.run_dir/'observations.jsonl').open('w')
        self.cell_log.writeheader(); self.vehicle_log.writeheader()
        self.create_timer(1.0, self.watchdog)
        self.get_logger().info(f'Logs: {self.run_dir}; admission is advisory only.')

    def watchdog(self):
        if self.last_seen is None or time.monotonic()-self.last_seen > 2.0:
            self.error = 'Waiting for fresh Gazebo observations; geometry commands paused.'
            if self.latest:
                stale = dict(self.latest, observations_stale=True)
                stale['cells'] = [dict(c, admission_advisory=False, admission_recommended=False, observation_fresh=False, reason='simulation_reset' if self.reset_latched else 'stale_observations')
                                  for c in self.latest['cells']]
                stale['observation_age_wall_s'] = None if self.last_seen is None else time.monotonic()-self.last_seen
                self.events_file.write(json.dumps(stale, allow_nan=False)+'\n'); self.events_file.flush()
                self.latest = stale
                self.state_pub.publish(String(data=json.dumps(stale, allow_nan=False)))
            if time.monotonic()-self.previous_warning > 5.0:
                self.get_logger().warning(self.error)
                self.previous_warning = time.monotonic()

    def on_observation(self, msg):
        try:
            if self.reset_latched:
                return
            obs = json.loads(msg.data)
            t = float(obs['sim_time'])
            if not math.isfinite(t):
                raise ValueError('Nonfinite simulation time')
            if self.last_t is not None and t <= self.last_t:
                if t < self.last_t:
                    self.reset_latched = True
                    self.last_seen = None
                    self.watchdog()
                    raise ValueError('Simulation reset detected. Restart the launch for a new run.')
                return
            incoming = {x['id']:x for x in obs['cells']}
            if set(incoming) != set(self.cells):
                raise ValueError('Observation cell IDs do not match this experiment')
            expected_names = {f'x500_{i}' for i in range(self.cfg['drones'])}
            vehicles = obs['vehicles']
            for v in vehicles:
                if v['name'] not in expected_names:
                    raise ValueError('Unexpected vehicle in observations')
                for field in ('position', 'velocity', 'wind', 'force'):
                    if len(v[field]) != 3 or not all(math.isfinite(x) for x in v[field]):
                        raise ValueError('Invalid vehicle vector')
            if len({v['name'] for v in vehicles}) != len(vehicles):
                raise ValueError('Duplicate vehicle observation')
            for key, measured in incoming.items():
                if (len(measured['wind']) != 3 or not all(math.isfinite(v) for v in measured['wind'])
                        or not math.isfinite(measured['width']) or measured['width'] <= 0):
                    raise ValueError('Invalid cell observation')
            complete = {v['name'] for v in vehicles} == expected_names
            self.raw_file.write(json.dumps(obs, allow_nan=False)+'\n'); self.raw_file.flush()
            self.last_t = t; self.last_seen = time.monotonic(); self.error = ''
            states = []
            for key, cell in self.cells.items():
                measured = incoming[key]
                width = float(measured['width'])
                wind = measured['wind']
                if len(wind) != 3 or not all(math.isfinite(x) for x in wind) or not math.isfinite(width):
                    raise ValueError('Invalid cell observation')
                occupants = [v for v in vehicles if occupies_station(cell, v['position'])]
                c = self.cfg['controller']
                occupied = max([abs(local_coordinates(cell, v['position'])[1]) +
                                c['vehicle_radius_m'] + c['extra_margin_m'] for v in occupants] or [0.0])
                if not complete:
                    occupied = max(occupied, c['max_width_m']/2)
                in_domain = True
                table = self.cfg.get('envelope_table')
                if c['model'] == 'table' and table:
                    lo, hi = table['speed_range_m_s']
                    along_wind = math.cos(cell['yaw'])*wind[0] + math.sin(cell['yaw'])*wind[1]
                    in_domain = (abs(along_wind) <= table['alongwind_limit_m_s'] and
                                 abs(wind[2]) <= table['vertical_wind_limit_m_s'] and
                                 all(lo <= math.sqrt(sum(x*x for x in v['velocity'])) <= hi for v in occupants))
                result = self.policies[key].update(t, crosswind(cell, wind), occupied, in_domain)
                self.commands[key].publish(Float64(data=float(result.commanded_width)))
                required_gap = max(0.0, result.required_width-width)
                advisory = (complete and result.admission_recommended and required_gap <= 1e-6
                            and width >= 2*occupied and measured['width_command_fresh'] and measured['panels_found'])
                reason = ('missing_aircraft' if not complete else 'missing_panels' if not measured['panels_found'] else
                          'stale_width_command' if not measured['width_command_fresh'] else
                          'applied_width_deficit' if required_gap > 1e-6 else result.reason)
                state = dict(id=key, **asdict(result), applied_width=width,
                             required_minus_applied=required_gap, occupants=len(occupants),
                             admission_advisory=advisory, panels_found=measured['panels_found'])
                if result.reason in ('outside_model_domain', 'insufficient_available_width'):
                    reason = result.reason
                state.update(model_domain_valid=result.reason != 'outside_model_domain', required_exceeds_available=result.required_width > c['max_width_m'], reason=reason, width_deficit=required_gap, observation_fresh=True, vehicles_complete=complete, wind_command_fresh=measured.get('wind_command_fresh', False), admission_enforced=False)
                states.append(state)
                self.cell_log.writerow(dict(sim_time=t, cell_id=key, policy=c['policy'], model=c['model'],
                    crosswind=result.observed_crosswind, design_crosswind=result.design_crosswind,
                    error_envelope=result.error_envelope, required_width=result.required_width,
                    commanded_width=result.commanded_width, applied_width=width,
                    required_minus_applied=required_gap, width_deficit=required_gap, model_domain_valid=result.reason != 'outside_model_domain', required_exceeds_available=result.required_width > c['max_width_m'], observation_fresh=True, vehicles_complete=complete, wind_command_fresh=measured.get('wind_command_fresh', False), occupants=len(occupants),
                    admission_advisory=advisory, reason=reason,
                    panels_found=measured['panels_found']))
                for v in vehicles:
                    along, cross, up = local_coordinates(cell, v['position'])
                    if not -cell['length']/2 <= along < cell['length']/2:
                        continue
                    local_cross = crosswind(cell, v['wind'])
                    along_wind = math.cos(cell['yaw'])*v['wind'][0]+math.sin(cell['yaw'])*v['wind'][1]
                    self.vehicle_log.writerow(dict(sim_time=t, vehicle=v['name'], cell_id=key,
                        along_m=along, cross_error_m=cross, vertical_error_m=up,
                        speed_m_s=math.sqrt(sum(x*x for x in v['velocity'])), wind_cross_m_s=local_cross,
                        wind_along_m_s=along_wind, wind_up_m_s=v['wind'][2],
                        force_n=math.sqrt(sum(x*x for x in v['force'])), force_limited=v['force_limited'],
                        applied_width=width, body_clearance_m=width/2-abs(cross)-c['vehicle_radius_m'],
                        model_clearance_m=width/2-(c['vehicle_radius_m']+result.error_envelope+c['extra_margin_m']),
                        in_altitude_band=abs(up) <= cell['height']/2, model=c['model'],
                        **{f'{field}_{axis}': value for field in ('position','velocity','wind','force') for axis,value in zip('xyz',v[field])}))
            self.latest = dict(sim_time=t, cells=states, vehicles=vehicles,
                               policy=self.cfg['controller']['policy'], admission_enforced=False,
                               observations_stale=False,
                               state_valid_for_sim_s=self.cfg['controller']['observation_timeout_s'])
            self.state_pub.publish(String(data=json.dumps(self.latest, allow_nan=False)))
            self.cell_file.flush(); self.vehicle_file.flush()
        except (ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)
            self.last_seen = None
            self.watchdog()
            self.get_logger().error(self.error)

    def close_logs(self):
        self.cell_file.close(); self.vehicle_file.close(); self.raw_file.close(); self.events_file.close()


def display(node):
    import tkinter as tk
    root = tk.Tk(); root.title('Wind-adaptive air cells | geometry experiment')
    canvas = tk.Canvas(root, width=1100, height=640, bg='#10202e', highlightthickness=0)
    canvas.pack(fill='both', expand=True)
    def draw():
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.0)
        canvas.delete('all')
        width, height = max(canvas.winfo_width(), 800), max(canvas.winfo_height(), 550)
        label = 'DEMO coefficients — not calibrated' if node.cfg['controller']['model']=='demo' else 'EMPIRICAL envelope — validation required'
        canvas.create_text(25, 25, anchor='nw', fill='#ffffff', font=('Arial', 17, 'bold'),
                           text='Individual air-cell width adaptation')
        canvas.create_text(25, 58, anchor='nw', fill='#ffca7a', font=('Arial', 11),
                           text=label+' | Ideal wind observations | Admission ADVISORY ONLY')
        if node.error:
            canvas.create_text(25, 92, anchor='nw', fill='#ff7979', text=node.error)
        latest = node.latest
        states = {x['id']:x for x in latest['cells']} if latest else {}
        n = len(node.cells); panel = (width-40)/n
        for i, (key, cell) in enumerate(node.cells.items()):
            x0 = 20+i*panel; cx = x0+panel/2; cy = 310
            scale = min((panel-35)/cell['length'], 190/cell['available_width'])
            l = cell['length']*scale
            state = states.get(key)
            actual = state['applied_width'] if state else node.cfg['initial_width_m']
            outer = cell['available_width']*scale
            h = actual*scale
            canvas.create_text(cx, 145, fill='white', text=key, font=('Arial', 12, 'bold'))
            canvas.create_rectangle(cx-l/2, cy-outer/2, cx+l/2, cy+outer/2,
                                    outline='#8195a5', dash=(5, 4), width=2)
            color = '#f58358' if state and state['required_minus_applied']>0 else '#58d5bd'
            canvas.create_rectangle(cx-l/2, cy-h/2, cx+l/2, cy+h/2, outline=color, width=3)
            canvas.create_line(cx-l/2, cy, cx+l/2, cy, fill='#8096a6', dash=(3, 5))
            if latest:
                for v in latest['vehicles']:
                    a, b, u = local_coordinates(cell, v['position'])
                    if -cell['length']/2 <= a < cell['length']/2:
                        px, py = cx+a*scale, cy-b*scale
                        canvas.create_oval(px-5,py-5,px+5,py+5,fill='#ffd25a',outline='')
                        canvas.create_text(px,py-15,fill='white',text=v['name'])
            if state:
                lines = [f"Crosswind: {state['observed_crosswind']:.2f} m/s",
                         f"Required width: {state['required_width']:.2f} m",
                         f"Commanded width: {state['commanded_width']:.2f} m",
                         f"Applied width: {actual:.2f} m",
                         f"Required − applied: {state['required_minus_applied']:.2f} m",
                         f"Entry advisory: {'OPEN' if state['admission_advisory'] else 'CLOSED'}",
                         f"{state['reason']}"]
                canvas.create_text(cx, 425, fill='white', text='\n'.join(lines), justify='left',
                                   font=('Arial', 11), anchor='n')
        canvas.create_text(25,height-30,anchor='w',fill='#a9c1d1',text=
            'Top view: travel →. Dashed = fixed available envelope. Solid = changing cell. Yellow = measured drone position.')
        root.after(80, draw)
    root.after(80, draw)
    root.mainloop()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--runtime', default=str(Path(__file__).resolve().parent/'run/runtime.json'))
    p.add_argument('--policy', choices=['adaptive','fixed_nominal','fixed_worst'])
    p.add_argument('--no-gui', action='store_true')
    args = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init(); node = CellManager(args.runtime, args.policy)
    try:
        if args.no_gui:
            rclpy.spin(node)
        else:
            display(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_logs(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
