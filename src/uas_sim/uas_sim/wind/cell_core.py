"""Pure cell geometry and width policy. No ROS / Gazebo dependencies.

The DEMO envelope is a tunable illustration, not evidence of safe containment.
Admission flags are recommendations, not enforced reservations.
"""
from collections import deque
from dataclasses import dataclass
import bisect
import math


def local_coordinates(cell, xyz):
    yaw = cell['yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xyz[0] - cell['x'], xyz[1] - cell['y']
    return c * dx + s * dy, -s * dx + c * dy, xyz[2] - cell['z']


def crosswind(cell, wind):
    return -math.sin(cell['yaw']) * wind[0] + math.cos(cell['yaw']) * wind[1]


def occupies_station(cell, xyz):
    along, _, up = local_coordinates(cell, xyz)
    # Width deliberately does not affect ownership: a vehicle outside the
    # changing width must still be counted as an occupant / violation.
    return (-cell['length'] / 2 <= along < cell['length'] / 2
            and abs(up) <= cell['height'] / 2)


@dataclass
class WidthResult:
    observed_crosswind: float
    design_crosswind: float
    error_envelope: float
    required_width: float
    commanded_width: float
    expansion_deficit: float
    admission_recommended: bool
    reason: str


class WidthPolicy:
    def __init__(self, config, initial_width, table=None):
        self.c = dict(config)
        if self.c['policy'] not in ('adaptive', 'fixed_nominal', 'fixed_worst'):
            raise ValueError('Unknown policy')
        if self.c['model'] not in ('demo', 'table'):
            raise ValueError('Unknown envelope model')
        for name in ('min_width_m', 'max_width_m', 'expand_rate_m_s',
                     'contract_rate_m_s', 'observation_timeout_s'):
            if not math.isfinite(self.c[name]) or self.c[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.c['min_width_m'] > self.c['max_width_m']:
            raise ValueError('Minimum width exceeds maximum')
        self.width = min(self.c['max_width_m'], max(self.c['min_width_m'], initial_width))
        self.samples = deque()
        self.last_t = None
        self.low_since = None
        self.table = table
        if self.c['model'] == 'table':
            if not table or not table.get('bins'):
                raise ValueError('Table mode requires a fitted envelope table')
            self.edges = [row['upper_crosswind_m_s'] for row in table['bins']]
            self.errors = [row['error_bound_m'] for row in table['bins']]
            if (any(not math.isfinite(v) or v < 0 for v in self.edges + self.errors)
                    or any(a >= b for a, b in zip(self.edges, self.edges[1:]))
                    or any(a > b for a, b in zip(self.errors, self.errors[1:]))):
                raise ValueError('Envelope table must be finite and monotone')

    def envelope(self, wind):
        if self.c['model'] == 'demo':
            value = self.c['demo_error_base_m'] + self.c['demo_error_coefficient'] * wind ** 2
            return value, wind <= self.c['max_calibrated_crosswind_m_s']
        if wind < self.table.get('crosswind_range_m_s', [0])[0]:
            return self.errors[0], False
        index = bisect.bisect_left(self.edges, wind)
        if index >= len(self.edges):
            return self.errors[-1], False
        return self.errors[index], True

    def update(self, t, cross, occupied_half_width=0.0, speed_in_domain=True):
        if not all(math.isfinite(v) for v in (t, cross, occupied_half_width)):
            raise ValueError('Nonfinite controller input')
        dt = 0.0 if self.last_t is None else t - self.last_t
        if dt < 0:
            raise ValueError('Simulation time reset; restart the experiment and logger')
        gap = dt > self.c['observation_timeout_s']
        self.last_t = t
        self.samples.append((t, abs(cross)))
        while len(self.samples) > 1 and self.samples[0][0] < t - self.c['wind_memory_s']:
            self.samples.popleft()
        design = max(v for _, v in self.samples)
        error, valid_wind = self.envelope(design)
        required = max(self.c['min_width_m'], 2 * (
            self.c['vehicle_radius_m'] + error + self.c['extra_margin_m']))
        observed_need = 2 * max(0, occupied_half_width)
        valid = valid_wind and speed_in_domain
        if self.c['policy'] == 'fixed_nominal':
            self.width = self.c['min_width_m']
        elif self.c['policy'] == 'fixed_worst':
            self.width = self.c['max_width_m']
        else:
            target = self.c['max_width_m'] if not valid else min(
                self.c['max_width_m'], max(required, observed_need))
            if gap:
                self.low_since = None
                dt = 0.0  # Never catch up an actuator through a telemetry outage.
            if target > self.width:
                self.width = min(target, self.width + self.c['expand_rate_m_s'] * dt)
                self.low_since = None
            elif target < self.width - self.c['hysteresis_m']:
                if self.low_since is None:
                    self.low_since = t
                if t - self.low_since >= self.c['contraction_hold_s']:
                    self.width = max(target, self.width - self.c['contract_rate_m_s'] * dt)
            else:
                self.low_since = None
        deficit = max(0.0, max(required, observed_need) - self.width)
        reason = ('outside_model_domain' if not valid else
                  'insufficient_available_width' if required > self.c['max_width_m'] else
                  'observation_gap' if gap else
                  'margin_deficit' if deficit > 1e-6 else 'width_sufficient')
        return WidthResult(abs(cross), design, error, required, self.width,
                           deficit, reason == 'width_sufficient', reason)
