import copy
import json
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cell_core import WidthPolicy, crosswind, local_coordinates, occupies_station

CONFIG=json.loads((Path(__file__).resolve().parents[1]/'config.json').read_text())['controller']


class CoreTests(unittest.TestCase):
    def test_projection_and_unique_ownership(self):
        a={'x':0,'y':0,'z':120,'yaw':0,'length':180,'height':50}
        b=dict(a,x=180)
        self.assertEqual(local_coordinates(a,[3,4,125]),(3,4,5))
        self.assertEqual(crosswind(a,[7,2,0]),2)
        self.assertFalse(occupies_station(a,[90,0,120]))
        self.assertTrue(occupies_station(b,[90,0,120]))
        # Large lateral error must not make an aircraft disappear from ownership.
        self.assertTrue(occupies_station(a,[0,100,120]))

    def test_wind_rotation(self):
        c={'yaw':-0.997447349598712}
        theta=c['yaw']; w=[-6*math.sin(theta),6*math.cos(theta),0]
        self.assertAlmostEqual(crosswind(c,w),6)
        self.assertAlmostEqual(crosswind(c,[math.cos(theta)*6,math.sin(theta)*6,0]),0)

    def test_rise_deficit_not_hidden(self):
        p=WidthPolicy(CONFIG,20);p.update(0,0)
        r=p.update(0.1,6)
        self.assertAlmostEqual(r.required_width,29.7)
        self.assertAlmostEqual(r.commanded_width,20.4)
        self.assertGreater(r.expansion_deficit,9)
        self.assertFalse(r.admission_recommended)

    def test_contraction_hold_and_slew(self):
        p=WidthPolicy(CONFIG,40);p.update(0,0)
        for i in range(1,100):
            self.assertEqual(p.update(i/10,0).commanded_width,40)
        before=p.width
        r=p.update(10,0)
        self.assertAlmostEqual(before-r.commanded_width,0.05)

    def test_current_occupant_blocks_shrink(self):
        p=WidthPolicy(CONFIG,40)
        for i in range(150):r=p.update(i/10,0,occupied_half_width=20)
        self.assertEqual(r.commanded_width,40)

    def test_max_width_and_domain(self):
        p=WidthPolicy(CONFIG,50); r=p.update(0,10)
        self.assertGreater(r.required_width,50)
        self.assertFalse(r.admission_recommended)
        self.assertEqual(r.reason,'outside_model_domain')

    def test_gap_freezes_slew_and_reset_rejected(self):
        p=WidthPolicy(CONFIG,20);p.update(0,0)
        r=p.update(20,6)
        self.assertEqual(r.commanded_width,20)
        self.assertFalse(r.admission_recommended)
        with self.assertRaises(ValueError):p.update(19,6)

    def test_fixed_baselines_keep_real_deficit(self):
        c=dict(CONFIG,policy='fixed_nominal');r=WidthPolicy(c,20).update(0,6)
        self.assertEqual(r.commanded_width,20)
        self.assertGreater(r.expansion_deficit,9)
        c=dict(CONFIG,policy='fixed_worst');r=WidthPolicy(c,20).update(0,6)
        self.assertEqual(r.commanded_width,50)
        self.assertEqual(r.expansion_deficit,0)

    def test_empirical_table_has_no_extrapolation(self):
        table={'bins':[{'upper_crosswind_m_s':1,'error_bound_m':0.5},
                       {'upper_crosswind_m_s':3,'error_bound_m':1.5}]}
        p=WidthPolicy(dict(CONFIG,model='table'),20,table)
        self.assertEqual(p.envelope(2),(1.5,True))
        self.assertEqual(p.envelope(4),(1.5,False))


if __name__=='__main__':unittest.main()
