from numa import NumaMain
from poses import gen_numa2_legs, g8Stand, g8FeetDown, g8Crouch
from IK import Gaits
import unittest

class TestBBLoader(unittest.TestCase):

    def setUp(self):
        leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
        gaits = Gaits(leg_geom, leg1, leg2, leg3, leg4)
        self.n = NumaMain(gaits)
        self.n.MIN_ADC_NO_BB = 20
        self.n.cmdrAlive = True
        self.n.enable_bb_loader = True

    def test_startup(self):
        # We start in timeout mode (this decision might change)
        self.n.bb_loader_service(0, 0, 30)
        self.assertEqual(self.n.loader_timeout_mode, "timeout")

    def test_ok(self):
        # Loader is active
        self.n.loader_timeout_mode = "running"
        self.n.bb_loader_service(0, 0, 30)
        self.assertEqual(self.n.loader_timeout_mode, "running")

    def test_first_bb_detection(self):
        # We saw a single BB; begin counting loops
        self.n.loader_timeout_mode = "running"
        self.n.bb_loader_service(0, 0, 10)
        self.assertEqual(self.n.loader_timeout_mode, "running")

    def test_no_2nd_bb_detection(self):
        # We saw a single BB, revert back to regular running mode
        self.n.loader_timeout_mode = "running"
        self.n.bb_detect_adc_loopcnt = 1
        self.n.bb_loader_service(0, 0, 30)
        self.assertEqual(self.n.loader_timeout_mode, "running")
        self.assertEqual(self.n.bb_detect_adc_loopcnt, 0)

    def test_20th_bb_detection(self):
        # We've seen bb's for several loops and go into timeout
        self.n.loader_timeout_mode = "running"
        self.n.bb_detect_adc_loopcnt = 19
        self.n.bb_loader_service(0, 0, 10)
        self.assertEqual(self.n.loader_timeout_mode, "timeout")

    def test_2nd_bb_detection_and_jam(self):
        # Expect to immediately go into jammed mode 
        self.n.loader_timeout_mode = "running"
        self.n.bb_detect_adc_loopcnt = 1
        self.n.bb_loader_service(0, 120, 30)
        self.assertEqual(self.n.loader_timeout_mode, "jammed")
        self.assertEqual(self.n.bb_detect_adc_loopcnt, 0)


        # Exiting timeout, but still detect BBs; expect that we don't enter running mode
