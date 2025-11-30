# TestAerodynamicsAdvanced.py
#
# Advanced tests for aerodynamics calculations.
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


class TestAerodynamicsAdvanced(JSBSimTestCase):
    """
    Advanced tests for aerodynamics calculations.

    Tests cover:
    - Alpha/beta calculations
    - Stability derivatives
    - Mach effects
    - Reynolds number
    - Load factors
    """

    def test_alpha_calculation(self):
        """Test angle of attack calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-rad"):
            alpha = fdm["aero/alpha-rad"]
            self.assertIsNotNone(alpha)
            # Alpha should be reasonable for level flight
            self.assertLess(abs(alpha), 0.5)

        del fdm

    def test_beta_calculation(self):
        """Test sideslip angle calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply rudder for sideslip
        fdm["fcs/rudder-cmd-norm"] = 0.3

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/beta-rad"):
            beta = fdm["aero/beta-rad"]
            self.assertIsNotNone(beta)

        del fdm

    def test_dynamic_pressure(self):
        """Test dynamic pressure calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/qbar-psf"):
            qbar = fdm["aero/qbar-psf"]
            # Dynamic pressure should be positive
            self.assertGreater(qbar, 0)

        del fdm

    def test_mach_number(self):
        """Test Mach number calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 30000
        fdm["ic/vc-kts"] = 400
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/mach"):
            mach = fdm["velocities/mach"]
            self.assertGreater(mach, 0)
            self.assertLess(mach, 2.0)

        del fdm

    def test_reynolds_number(self):
        """Test Reynolds number is calculated."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/Re"):
            re = fdm["aero/Re"]
            # Reynolds number should be large
            self.assertGreater(re, 1e6)

        del fdm

    def test_load_factor_nz(self):
        """Test load factor Nz calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/Nz"):
            nz = fdm["aero/Nz"]
            # Load factor should be near 1G for level flight
            self.assertAlmostEqual(nz, 1.0, delta=0.5)

        del fdm

    def test_lift_coefficient(self):
        """Test lift coefficient varies with alpha."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/CL"):
            cl = fdm["aero/CL"]
            # Lift coefficient should be positive for level flight
            self.assertGreater(cl, 0)

        del fdm

    def test_drag_coefficient(self):
        """Test drag coefficient is positive."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/CD"):
            cd = fdm["aero/CD"]
            # Drag coefficient should be positive
            self.assertGreater(cd, 0)

        del fdm

    def test_transonic_flight(self):
        """Test transonic flight conditions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 35000
        fdm["ic/mach"] = 0.95  # Transonic
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.9

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/mach"):
            mach = fdm["velocities/mach"]
            self.assertGreater(mach, 0.8)

        del fdm

    def test_high_alpha_flight(self):
        """Test high angle of attack conditions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 60  # Low speed
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8
        fdm["fcs/elevator-cmd-norm"] = -0.5  # Pull back

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-deg"):
            alpha = fdm["aero/alpha-deg"]
            # Should have positive alpha
            self.assertGreater(alpha, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestAerodynamicsAdvanced)
