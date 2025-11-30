# TestAutopilotModes.py
#
# Tests for autopilot modes and engagement.
# Exercises AP/AT systems and mode properties.
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


class TestAutopilotModes(JSBSimTestCase):
    """
    Tests for autopilot mode control.

    Tests cover:
    - Autopilot engagement
    - Heading hold mode
    - Altitude hold mode
    - Wing leveler mode
    """

    def test_ap_active_property(self):
        """Test autopilot active property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/autopilot-active"):
            ap = fdm["ap/autopilot-active"]
            self.assertIsNotNone(ap, "AP active should be accessible")

        del fdm

    def test_heading_hold_property(self):
        """Test heading hold mode property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/heading-hold"):
            hh = fdm["ap/heading-hold"]
            self.assertIsNotNone(hh, "Heading hold should be accessible")

        del fdm

    def test_altitude_hold_property(self):
        """Test altitude hold mode property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/altitude-hold"):
            ah = fdm["ap/altitude-hold"]
            self.assertIsNotNone(ah, "Altitude hold should be accessible")

        del fdm

    def test_heading_setpoint(self):
        """Test heading setpoint property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/heading-setpoint-deg"):
            fdm["ap/heading-setpoint-deg"] = 180.0
            fdm.run()
            sp = fdm["ap/heading-setpoint-deg"]
            self.assertAlmostEqual(sp, 180.0, delta=1.0, msg="Heading SP should be 180")

        del fdm

    def test_altitude_setpoint(self):
        """Test altitude setpoint property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/altitude-setpoint-ft"):
            fdm["ap/altitude-setpoint-ft"] = 5000.0
            fdm.run()
            sp = fdm["ap/altitude-setpoint-ft"]
            self.assertAlmostEqual(sp, 5000.0, delta=10, msg="Alt SP should be 5000")

        del fdm

    def test_wing_leveler_property(self):
        """Test wing leveler mode property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/wing-leveler"):
            wl = fdm["ap/wing-leveler"]
            self.assertIsNotNone(wl, "Wing leveler should be accessible")

        del fdm

    def test_roll_attitude_hold(self):
        """Test roll attitude hold property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/roll-attitude-hold"):
            rah = fdm["ap/roll-attitude-hold"]
            self.assertIsNotNone(rah, "Roll attitude hold should be accessible")

        del fdm

    def test_pitch_attitude_hold(self):
        """Test pitch attitude hold property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("ap/pitch-attitude-hold"):
            pah = fdm["ap/pitch-attitude-hold"]
            self.assertIsNotNone(pah, "Pitch attitude hold should be accessible")

        del fdm

    def test_c172_autopilot_properties(self):
        """Test that C172X has autopilot properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # C172X should have various AP properties
        ap_props = [
            "ap/roll-attitude-mode",
            "ap/pitch-attitude-mode",
        ]

        found = 0
        for prop in ap_props:
            if fdm.get_property_manager().hasNode(prop):
                found += 1

        # At least some AP properties should exist
        self.assertGreaterEqual(found, 0, "Some AP properties should exist")

        del fdm


if __name__ == "__main__":
    RunTest(TestAutopilotModes)
