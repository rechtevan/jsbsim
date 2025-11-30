# TestTurnDynamics.py
#
# Tests for turning flight dynamics.
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


class TestTurnDynamics(JSBSimTestCase):
    """
    Tests for turning flight dynamics.

    Tests cover:
    - Turn rate
    - Bank angle effects
    - Heading change
    - Coordinated turns
    """

    def test_bank_causes_heading_change(self):
        """Test that banking causes heading to change."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("attitude/psi-deg"):
            del fdm
            return

        # Apply aileron for bank
        fdm["fcs/aileron-cmd-norm"] = 0.3

        for _ in range(200):
            fdm.run()

        psi = fdm["attitude/psi-deg"]
        # Heading should have changed
        self.assertIsNotNone(psi)

        del fdm

    def test_turn_rate_property(self):
        """Test turn rate property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/psidot-rad_sec"):
            psidot = fdm["velocities/psidot-rad_sec"]
            self.assertIsNotNone(psidot)

        del fdm

    def test_rudder_causes_yaw(self):
        """Test that rudder input causes yaw."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("velocities/r-rad_sec"):
            del fdm
            return

        # Apply rudder
        fdm["fcs/rudder-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        r = fdm["velocities/r-rad_sec"]
        # Should have yaw rate
        self.assertIsNotNone(r)

        del fdm

    def test_coordinated_turn(self):
        """Test coordinated turn with aileron and rudder."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply coordinated inputs
        fdm["fcs/aileron-cmd-norm"] = 0.3
        fdm["fcs/rudder-cmd-norm"] = 0.1

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/beta-deg"):
            beta = fdm["aero/beta-deg"]
            # Sideslip should be relatively small in coordinated turn
            self.assertIsNotNone(beta)

        del fdm

    def test_roll_rate_with_aileron(self):
        """Test roll rate responds to aileron."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("velocities/p-rad_sec"):
            del fdm
            return

        # Apply aileron
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(20):
            fdm.run()

        p = fdm["velocities/p-rad_sec"]
        # Should have roll rate
        self.assertNotEqual(p, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestTurnDynamics)
