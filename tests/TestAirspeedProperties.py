# TestAirspeedProperties.py
#
# Tests for airspeed-related properties.
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


class TestAirspeedProperties(JSBSimTestCase):
    """
    Tests for airspeed properties.

    Tests cover:
    - True airspeed
    - Calibrated airspeed
    - Equivalent airspeed
    - Groundspeed
    """

    def test_true_airspeed_kts(self):
        """Test true airspeed in knots."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vt-kts"):
            vt = fdm["velocities/vt-kts"]
            self.assertGreater(vt, 0)

        del fdm

    def test_calibrated_airspeed(self):
        """Test calibrated airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vc-kts"):
            vc = fdm["velocities/vc-kts"]
            self.assertAlmostEqual(vc, 100, delta=10)

        del fdm

    def test_equivalent_airspeed(self):
        """Test equivalent airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/ve-kts"):
            ve = fdm["velocities/ve-kts"]
            self.assertIsNotNone(ve)

        del fdm

    def test_groundspeed(self):
        """Test groundspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            self.assertGreater(vg, 0)

        del fdm

    def test_airspeed_fps(self):
        """Test true airspeed in fps."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vt-fps"):
            vt = fdm["velocities/vt-fps"]
            self.assertGreater(vt, 0)

        del fdm

    def test_mach_number(self):
        """Test Mach number property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/mach"):
            mach = fdm["velocities/mach"]
            # At 100 kts, Mach should be around 0.15
            self.assertGreater(mach, 0)
            self.assertLess(mach, 0.5)

        del fdm

    def test_airspeed_zero_at_rest(self):
        """Test airspeed is zero when stationary."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/u-fps"] = 0
        fdm["ic/v-fps"] = 0
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vt-fps"):
            vt = fdm["velocities/vt-fps"]
            self.assertAlmostEqual(vt, 0, delta=1)

        del fdm


if __name__ == "__main__":
    RunTest(TestAirspeedProperties)
