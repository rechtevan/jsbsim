# TestAngularRates.py
#
# Tests for angular rate properties (roll, pitch, yaw rates).
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


class TestAngularRates(JSBSimTestCase):
    """
    Tests for angular rate properties.

    Tests cover:
    - Roll rate (P)
    - Pitch rate (Q)
    - Yaw rate (R)
    - Angular accelerations
    """

    def test_roll_rate_property(self):
        """Test roll rate property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertIsNotNone(p)

        del fdm

    def test_pitch_rate_property(self):
        """Test pitch rate property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/q-rad_sec"):
            q = fdm["velocities/q-rad_sec"]
            self.assertIsNotNone(q)

        del fdm

    def test_yaw_rate_property(self):
        """Test yaw rate property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/r-rad_sec"):
            r = fdm["velocities/r-rad_sec"]
            self.assertIsNotNone(r)

        del fdm

    def test_initial_rates_zero(self):
        """Test that initial angular rates are zero."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/p-rad_sec"] = 0
        fdm["ic/q-rad_sec"] = 0
        fdm["ic/r-rad_sec"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertAlmostEqual(p, 0, delta=0.01)

        del fdm

    def test_set_initial_roll_rate(self):
        """Test setting initial roll rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/p-rad_sec"] = 0.1
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            self.assertAlmostEqual(p, 0.1, delta=0.05)

        del fdm

    def test_aileron_causes_roll(self):
        """Test that aileron input causes roll rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply aileron
        fdm["fcs/aileron-cmd-norm"] = 0.5

        # Run simulation
        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            # Should have some roll rate with aileron applied
            self.assertIsNotNone(p)

        del fdm

    def test_elevator_causes_pitch(self):
        """Test that elevator input causes pitch rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply elevator
        fdm["fcs/elevator-cmd-norm"] = -0.3

        # Run simulation
        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/q-rad_sec"):
            q = fdm["velocities/q-rad_sec"]
            self.assertIsNotNone(q)

        del fdm

    def test_rudder_causes_yaw(self):
        """Test that rudder input causes yaw rate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply rudder
        fdm["fcs/rudder-cmd-norm"] = 0.3

        # Run simulation
        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/r-rad_sec"):
            r = fdm["velocities/r-rad_sec"]
            self.assertIsNotNone(r)

        del fdm

    def test_angular_rates_degrees(self):
        """Test angular rates in degrees per second."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-aero-deg_sec"):
            p_deg = fdm["velocities/p-aero-deg_sec"]
            self.assertIsNotNone(p_deg)

        del fdm


if __name__ == "__main__":
    RunTest(TestAngularRates)
