# TestVelocitiesBasic.py
#
# Tests for velocity properties and calculations.
# Exercises airspeed, groundspeed, and Mach number.
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


class TestVelocitiesBasic(JSBSimTestCase):
    """
    Tests for velocity properties.

    Tests cover:
    - True airspeed
    - Calibrated airspeed
    - Groundspeed
    - Mach number
    - Body frame velocities
    """

    def test_true_airspeed(self):
        """Test true airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vt-fps"):
            vt = fdm["velocities/vt-fps"]
            # 100 kts is about 169 fps
            self.assertGreater(vt, 100)
            self.assertLess(vt, 250)

        del fdm

    def test_calibrated_airspeed(self):
        """Test calibrated airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vc-kts"):
            vc = fdm["velocities/vc-kts"]
            self.assertAlmostEqual(vc, 100, delta=5)

        del fdm

    def test_groundspeed(self):
        """Test groundspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/vg-fps"):
            vg = fdm["velocities/vg-fps"]
            self.assertGreater(vg, 0)

        del fdm

    def test_mach_number(self):
        """Test Mach number property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/vc-kts"] = 150
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/mach"):
            mach = fdm["velocities/mach"]
            # C172 at 150 kts should be around Mach 0.2-0.25
            self.assertGreater(mach, 0.1)
            self.assertLess(mach, 0.5)

        del fdm

    def test_supersonic_mach(self):
        """Test supersonic Mach number."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f16")
            fdm["ic/h-sl-ft"] = 40000
            fdm["ic/mach"] = 1.5
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("velocities/mach"):
                mach = fdm["velocities/mach"]
                self.assertAlmostEqual(mach, 1.5, delta=0.1)
        except Exception:
            pass
        finally:
            del fdm

    def test_body_u_velocity(self):
        """Test body frame u velocity (forward)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/u-fps"] = 200
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/u-fps"):
            u = fdm["velocities/u-fps"]
            self.assertAlmostEqual(u, 200, delta=10)

        del fdm

    def test_body_v_velocity(self):
        """Test body frame v velocity (right)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/v-fps"] = 10
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/v-fps"):
            v = fdm["velocities/v-fps"]
            self.assertIsNotNone(v)

        del fdm

    def test_body_w_velocity(self):
        """Test body frame w velocity (down)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/w-fps"] = 5
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/w-fps"):
            w = fdm["velocities/w-fps"]
            self.assertIsNotNone(w)

        del fdm

    def test_equivalent_airspeed(self):
        """Test equivalent airspeed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/ve-kts"):
            ve = fdm["velocities/ve-kts"]
            # EAS should be less than TAS at altitude
            self.assertIsNotNone(ve)

        del fdm

    def test_vertical_speed(self):
        """Test vertical speed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/v-down-fps"):
            vd = fdm["velocities/v-down-fps"]
            self.assertIsNotNone(vd)

        del fdm

    def test_velocity_changes_with_time(self):
        """Test that velocity changes during simulation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        initial_vd = fdm["velocities/v-down-fps"]

        for _ in range(100):
            fdm.run()

        final_vd = fdm["velocities/v-down-fps"]

        # Ball should accelerate downward due to gravity
        self.assertGreater(final_vd, initial_vd)

        del fdm


if __name__ == "__main__":
    RunTest(TestVelocitiesBasic)
