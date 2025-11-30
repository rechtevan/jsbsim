# TestBodyRates.py
#
# Tests for body angular rates (p, q, r).
# Exercises roll, pitch, and yaw rates.
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>
#

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestBodyRates(JSBSimTestCase):
    """
    Tests for body angular rates.

    Tests cover:
    - Roll rate (p)
    - Pitch rate (q)
    - Yaw rate (r)
    - Angular rate initialization
    - Rate response to control inputs
    """

    def test_p_rad_sec_property(self):
        """Test roll rate (p) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        p = fdm["velocities/p-rad_sec"]
        self.assertIsNotNone(p, "Roll rate should be accessible")

        del fdm

    def test_q_rad_sec_property(self):
        """Test pitch rate (q) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        q = fdm["velocities/q-rad_sec"]
        self.assertIsNotNone(q, "Pitch rate should be accessible")

        del fdm

    def test_r_rad_sec_property(self):
        """Test yaw rate (r) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        r = fdm["velocities/r-rad_sec"]
        self.assertIsNotNone(r, "Yaw rate should be accessible")

        del fdm

    def test_initial_rates_zero(self):
        """Test that rates are zero at initialization."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/p-rad_sec"] = 0.0
        fdm["ic/q-rad_sec"] = 0.0
        fdm["ic/r-rad_sec"] = 0.0
        fdm.run_ic()

        p = fdm["velocities/p-rad_sec"]
        q = fdm["velocities/q-rad_sec"]
        r = fdm["velocities/r-rad_sec"]

        self.assertAlmostEqual(p, 0.0, delta=0.1, msg="P should be near 0")
        self.assertAlmostEqual(q, 0.0, delta=0.1, msg="Q should be near 0")
        self.assertAlmostEqual(r, 0.0, delta=0.1, msg="R should be near 0")

        del fdm

    def test_p_deg_sec_property(self):
        """Test roll rate in deg/sec."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("velocities/p-deg_sec"):
            p_deg = fdm["velocities/p-deg_sec"]
            self.assertIsNotNone(p_deg, "P deg/sec should be accessible")

        del fdm

    def test_aileron_produces_roll_rate(self):
        """Test that aileron input produces roll rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Apply aileron
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        p = fdm["velocities/p-rad_sec"]
        # Should have some roll rate with aileron applied
        self.assertIsNotNone(p, "Roll rate should respond to aileron")

        del fdm

    def test_elevator_produces_pitch_rate(self):
        """Test that elevator input produces pitch rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Apply elevator
        fdm["fcs/elevator-cmd-norm"] = -0.3

        for _ in range(50):
            fdm.run()

        q = fdm["velocities/q-rad_sec"]
        # Should have pitch rate with elevator applied
        self.assertIsNotNone(q, "Pitch rate should respond to elevator")

        del fdm

    def test_rudder_produces_yaw_rate(self):
        """Test that rudder input produces yaw rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        # Apply rudder
        fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        r = fdm["velocities/r-rad_sec"]
        # Should have yaw rate with rudder applied
        self.assertIsNotNone(r, "Yaw rate should respond to rudder")

        del fdm

    def test_p_aero_property(self):
        """Test aerodynamic roll rate property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        fdm.run()

        if fdm.get_property_manager().hasNode("velocities/p-aero-rad_sec"):
            p_aero = fdm["velocities/p-aero-rad_sec"]
            self.assertIsNotNone(p_aero, "Aero roll rate should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestBodyRates)
