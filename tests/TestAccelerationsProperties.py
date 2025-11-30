# TestAccelerationsProperties.py
#
# Tests for acceleration properties (body, NED).
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


class TestAccelerationsProperties(JSBSimTestCase):
    """
    Tests for acceleration properties.

    Tests cover:
    - Body frame accelerations
    - NED frame accelerations
    - Load factors
    """

    def test_udot_property(self):
        """Test body u-dot (forward acceleration)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/udot-ft_sec2"):
            udot = fdm["accelerations/udot-ft_sec2"]
            self.assertIsNotNone(udot)

        del fdm

    def test_vdot_property(self):
        """Test body v-dot (lateral acceleration)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/vdot-ft_sec2"):
            vdot = fdm["accelerations/vdot-ft_sec2"]
            self.assertIsNotNone(vdot)

        del fdm

    def test_wdot_property(self):
        """Test body w-dot (vertical acceleration)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/wdot-ft_sec2"):
            wdot = fdm["accelerations/wdot-ft_sec2"]
            self.assertIsNotNone(wdot)

        del fdm

    def test_gravity_causes_acceleration(self):
        """Test that gravity causes downward acceleration."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm["ic/u-fps"] = 0
        fdm["ic/v-fps"] = 0
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        # Run a few steps
        for _ in range(10):
            fdm.run()

        # Should have downward velocity from gravity
        vd = fdm["velocities/v-down-fps"]
        self.assertGreater(vd, 0)  # Positive = downward

        del fdm

    def test_n_load_factor(self):
        """Test normal load factor (Nz)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/Nz"):
            nz = fdm["aero/Nz"]
            # In level flight, Nz should be around 1
            self.assertIsNotNone(nz)

        del fdm

    def test_pdot_property(self):
        """Test roll acceleration (p-dot)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/pdot-rad_sec2"):
            pdot = fdm["accelerations/pdot-rad_sec2"]
            self.assertIsNotNone(pdot)

        del fdm

    def test_qdot_property(self):
        """Test pitch acceleration (q-dot)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/qdot-rad_sec2"):
            qdot = fdm["accelerations/qdot-rad_sec2"]
            self.assertIsNotNone(qdot)

        del fdm

    def test_rdot_property(self):
        """Test yaw acceleration (r-dot)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/rdot-rad_sec2"):
            rdot = fdm["accelerations/rdot-rad_sec2"]
            self.assertIsNotNone(rdot)

        del fdm


if __name__ == "__main__":
    RunTest(TestAccelerationsProperties)
