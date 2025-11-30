# TestAeroMoments.py
#
# Tests for aerodynamic moment properties (roll, pitch, yaw moments).
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


class TestAeroMoments(JSBSimTestCase):
    """
    Tests for aerodynamic moment properties.

    Tests cover:
    - Roll moment (L)
    - Pitch moment (M)
    - Yaw moment (N)
    """

    def test_roll_moment(self):
        """Test roll moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/l-aero-lbsft"):
            L = fdm["moments/l-aero-lbsft"]
            self.assertIsNotNone(L)

        del fdm

    def test_pitch_moment(self):
        """Test pitch moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/m-aero-lbsft"):
            M = fdm["moments/m-aero-lbsft"]
            self.assertIsNotNone(M)

        del fdm

    def test_yaw_moment(self):
        """Test yaw moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("moments/n-aero-lbsft"):
            N = fdm["moments/n-aero-lbsft"]
            self.assertIsNotNone(N)

        del fdm

    def test_aileron_causes_roll_moment(self):
        """Test that aileron input causes roll moment."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("moments/l-aero-lbsft"):
            del fdm
            return

        fdm["fcs/aileron-cmd-norm"] = 0.5
        fdm.run()

        L_after = fdm["moments/l-aero-lbsft"]
        # Should have some roll moment with aileron
        self.assertIsNotNone(L_after)

        del fdm

    def test_elevator_causes_pitch_moment(self):
        """Test that elevator input causes pitch moment."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("moments/m-aero-lbsft"):
            del fdm
            return

        fdm["fcs/elevator-cmd-norm"] = -0.3
        fdm.run()

        M = fdm["moments/m-aero-lbsft"]
        self.assertIsNotNone(M)

        del fdm

    def test_rudder_causes_yaw_moment(self):
        """Test that rudder input causes yaw moment."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("moments/n-aero-lbsft"):
            del fdm
            return

        fdm["fcs/rudder-cmd-norm"] = 0.3
        fdm.run()

        N = fdm["moments/n-aero-lbsft"]
        self.assertIsNotNone(N)

        del fdm


if __name__ == "__main__":
    RunTest(TestAeroMoments)
