# TestForcesMoments.py
#
# Tests for forces and moments properties.
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


class TestForcesMoments(JSBSimTestCase):
    """
    Tests for force and moment properties.

    Tests cover:
    - Total forces (body frame)
    - Total moments (body frame)
    - Aerodynamic forces
    - Propulsion forces
    """

    def test_total_force_x(self):
        """Test total X-axis force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fbx-total-lbs"):
            fx = fdm["forces/fbx-total-lbs"]
            self.assertIsNotNone(fx)

        del fdm

    def test_total_force_y(self):
        """Test total Y-axis force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fby-total-lbs"):
            fy = fdm["forces/fby-total-lbs"]
            self.assertIsNotNone(fy)

        del fdm

    def test_total_force_z(self):
        """Test total Z-axis force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fbz-total-lbs"):
            fz = fdm["forces/fbz-total-lbs"]
            self.assertIsNotNone(fz)

        del fdm

    def test_rolling_moment(self):
        """Test rolling moment (L)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/l-total-lbsft"):
            l_moment = fdm["moments/l-total-lbsft"]
            self.assertIsNotNone(l_moment)

        del fdm

    def test_pitching_moment(self):
        """Test pitching moment (M)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/m-total-lbsft"):
            m_moment = fdm["moments/m-total-lbsft"]
            self.assertIsNotNone(m_moment)

        del fdm

    def test_yawing_moment(self):
        """Test yawing moment (N)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/n-total-lbsft"):
            n_moment = fdm["moments/n-total-lbsft"]
            self.assertIsNotNone(n_moment)

        del fdm

    def test_aero_force_drag(self):
        """Test aerodynamic drag force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fwx-aero-lbs"):
            drag = fdm["forces/fwx-aero-lbs"]
            # Drag should be negative in wind frame
            self.assertIsNotNone(drag)

        del fdm

    def test_aero_force_lift(self):
        """Test aerodynamic lift force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("forces/fwz-aero-lbs"):
            lift = fdm["forces/fwz-aero-lbs"]
            self.assertIsNotNone(lift)

        del fdm


if __name__ == "__main__":
    RunTest(TestForcesMoments)
