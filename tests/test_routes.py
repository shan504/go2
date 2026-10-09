import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from route_analysis import connected,inscribed_radius,static_clearance


class RouteTests(unittest.TestCase):
    def test_padded_body_clearance_is_not_two_centimetres(self):
        radius=inscribed_radius([[-.4,-.22],[-.4,.22],[.4,.22],[.4,-.22]],.01)
        self.assertAlmostEqual(radius,.23)

    def test_free_endpoints_can_be_disconnected_by_body_clearance(self):
        data=np.zeros((30,50),dtype=np.int8)
        data[:,25]=100;data[12:18,25]=0
        self.assertTrue(connected(data,(10,15),(40,15)).startswith('YES'))
        band=static_clearance(data,.05,.23)
        self.assertEqual(band[15,10],0);self.assertEqual(band[15,40],0)
        self.assertTrue(connected(band,(10,15),(40,15)).startswith('NO'))

    def test_live_marks_and_unknown_are_distinct_from_static_clearance(self):
        data=np.zeros((30,50),dtype=np.int8)
        predicted=static_clearance(data,.05,.23)
        self.assertTrue(connected(predicted,(10,15),(40,15)).startswith('YES'))
        live=predicted.copy();live[:,25]=99
        self.assertTrue(connected(live,(10,15),(40,15)).startswith('NO'))
        data[:,25]=-1
        predicted=static_clearance(data,.05,.23)
        self.assertTrue((predicted[:,25]==-1).all())
        self.assertTrue(connected(predicted,(10,15),(40,15)).startswith('NO'))


if __name__=='__main__':
    unittest.main()
