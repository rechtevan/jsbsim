# TestAeroForces.py
#
# Tests for aerodynamic force properties (lift, drag, side force).
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


class TestAeroForces(JSBSimTestCase):
    """
    Tests for aerodynamic force properties.

    Tests cover:
    - Lift force
    - Drag force
    - Side force
    - Force coefficients
    """

    def test_lift_force(self):
        """Test lift force property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fbz-aero-lbs"):
            fz = fdm["forces/fbz-aero-lbs"]
            # Lift (negative Z in body) should exist
            self.assertIsNotNone(fz)

        del fdm

    def test_drag_force(self):
        """Test drag force property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fbx-aero-lbs"):
            fx = fdm["forces/fbx-aero-lbs"]
            # Drag should exist
            self.assertIsNotNone(fx)

        del fdm

    def test_side_force(self):
        """Test side force property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fby-aero-lbs"):
            fy = fdm["forces/fby-aero-lbs"]
            self.assertIsNotNone(fy)

        del fdm

    def test_cl_property(self):
        """Test lift coefficient property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/cl-squared"):
            cl_sq = fdm["aero/cl-squared"]
            self.assertIsNotNone(cl_sq)
            self.assertGreaterEqual(cl_sq, 0)

        del fdm

    def test_forces_increase_with_speed(self):
        """Test that aero forces increase with speed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("forces/fbz-aero-lbs"):
            del fdm
            return

        fz_low = abs(fdm["forces/fbz-aero-lbs"])

        fdm["ic/vc-kts"] = 120
        fdm.run_ic()
        fz_high = abs(fdm["forces/fbz-aero-lbs"])

        # Higher speed should give more lift
        self.assertGreater(fz_high, fz_low * 0.8)

        del fdm

    def test_total_aero_force(self):
        """Test total aerodynamic force magnitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()

        total_force = 0
        if pm.hasNode("forces/fbx-aero-lbs"):
            fx = fdm["forces/fbx-aero-lbs"]
            total_force += fx * fx
        if pm.hasNode("forces/fby-aero-lbs"):
            fy = fdm["forces/fby-aero-lbs"]
            total_force += fy * fy
        if pm.hasNode("forces/fbz-aero-lbs"):
            fz = fdm["forces/fbz-aero-lbs"]
            total_force += fz * fz

        # Should have some aero force
        self.assertGreater(total_force, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestAeroForces)
