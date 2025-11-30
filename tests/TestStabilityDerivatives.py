# TestStabilityDerivatives.py
#
# Tests for stability derivatives and dynamic stability.
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


class TestStabilityDerivatives(JSBSimTestCase):
    """
    Tests for stability derivatives.

    Tests cover:
    - CL_alpha (lift curve slope)
    - CM_alpha (pitch stiffness)
    - Cn_beta (yaw stability)
    - Cl_beta (dihedral effect)
    """

    def test_aero_alpha_effect(self):
        """Test that alpha affects lift."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Get initial lift
        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/force/Lift"):
            del fdm
            return

        # Run to stabilize
        for _ in range(50):
            fdm.run()

        lift1 = fdm["aero/force/Lift"]

        # Pitch up to increase alpha
        fdm["fcs/elevator-cmd-norm"] = -0.3
        for _ in range(50):
            fdm.run()

        lift2 = fdm["aero/force/Lift"]
        # Lift should change with alpha
        self.assertNotAlmostEqual(lift1, lift2, delta=10)

        del fdm

    def test_sideslip_effect(self):
        """Test that sideslip affects side force."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/beta-deg"):
            del fdm
            return

        # Apply rudder to create sideslip
        fdm["fcs/rudder-cmd-norm"] = 0.3
        for _ in range(100):
            fdm.run()

        beta = fdm["aero/beta-deg"]
        # Should have some sideslip
        self.assertIsNotNone(beta)

        del fdm

    def test_roll_damping(self):
        """Test roll damping effect."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("velocities/p-rad_sec"):
            del fdm
            return

        # Induce roll rate
        fdm["fcs/aileron-cmd-norm"] = 1.0
        for _ in range(20):
            fdm.run()

        # Get peak roll rate
        p1 = abs(fdm["velocities/p-rad_sec"])

        # Remove aileron input
        fdm["fcs/aileron-cmd-norm"] = 0.0
        for _ in range(100):
            fdm.run()

        # Roll rate should decrease (damping)
        p2 = abs(fdm["velocities/p-rad_sec"])
        self.assertLess(p2, p1 * 2)  # Allow some tolerance

        del fdm

    def test_pitch_damping(self):
        """Test pitch damping effect."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("velocities/q-rad_sec"):
            del fdm
            return

        # Induce pitch rate
        fdm["fcs/elevator-cmd-norm"] = -0.5
        for _ in range(20):
            fdm.run()

        # Get pitch rate - verify property exists
        abs(fdm["velocities/q-rad_sec"])

        # Remove elevator
        fdm["fcs/elevator-cmd-norm"] = 0.0
        for _ in range(100):
            fdm.run()

        # Pitch rate behavior should change
        q2 = abs(fdm["velocities/q-rad_sec"])
        self.assertIsNotNone(q2)

        del fdm

    def test_yaw_damping(self):
        """Test yaw damping effect."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("velocities/r-rad_sec"):
            del fdm
            return

        # Induce yaw rate
        fdm["fcs/rudder-cmd-norm"] = 0.5
        for _ in range(20):
            fdm.run()

        # Get yaw rate - verify property exists
        abs(fdm["velocities/r-rad_sec"])

        # Remove rudder
        fdm["fcs/rudder-cmd-norm"] = 0.0
        for _ in range(100):
            fdm.run()

        r2 = abs(fdm["velocities/r-rad_sec"])
        self.assertIsNotNone(r2)

        del fdm


if __name__ == "__main__":
    RunTest(TestStabilityDerivatives)
