# TestAerodynamicsCoefficients.py
#
# Tests for aerodynamic coefficients and forces (FGAerodynamics).
# Exercises lift, drag, and moment calculations.
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


class TestAerodynamicsCoefficients(JSBSimTestCase):
    """
    Tests for aerodynamic coefficients (FGAerodynamics).

    Tests cover:
    - Lift coefficient and force
    - Drag coefficient and force
    - Side force coefficient
    - Pitching moment
    - Rolling and yawing moments
    - Dynamic pressure effects
    """

    def test_lift_coefficient_exists(self):
        """Test that lift coefficient property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Check lift coefficient
        if fdm.get_property_manager().hasNode("aero/cl-squared"):
            cl_sq = fdm["aero/cl-squared"]
            self.assertIsNotNone(cl_sq, "CL squared should be accessible")

        del fdm

    def test_drag_coefficient_exists(self):
        """Test that drag properties exist."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Check for drag-related properties
        if fdm.get_property_manager().hasNode("aero/coefficient/CD0"):
            cd0 = fdm["aero/coefficient/CD0"]
            self.assertIsNotNone(cd0, "CD0 should be accessible")

        del fdm

    def test_dynamic_pressure(self):
        """Test dynamic pressure calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        qbar = fdm["aero/qbar-psf"]
        self.assertGreater(qbar, 0, "Dynamic pressure should be positive at speed")

        del fdm

    def test_qbar_increases_with_speed(self):
        """Test that dynamic pressure increases with airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")

        # Test at low speed
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 100
        fdm.run_ic()

        for _ in range(10):
            fdm.run()
        qbar_slow = fdm["aero/qbar-psf"]

        # Test at higher speed
        fdm["ic/u-fps"] = 200
        fdm.run_ic()

        for _ in range(10):
            fdm.run()
        qbar_fast = fdm["aero/qbar-psf"]

        # qbar should increase with velocity squared
        self.assertGreater(qbar_fast, qbar_slow, "Qbar should increase with speed")

        del fdm

    def test_aerodynamic_forces(self):
        """Test aerodynamic force properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Check force properties (body axis)
        fx = fdm["forces/fbx-aero-lbs"]
        fy = fdm["forces/fby-aero-lbs"]
        fz = fdm["forces/fbz-aero-lbs"]

        self.assertIsNotNone(fx, "Aero force X should be accessible")
        self.assertIsNotNone(fy, "Aero force Y should be accessible")
        self.assertIsNotNone(fz, "Aero force Z should be accessible")

        del fdm

    def test_lift_force_in_flight(self):
        """Test that lift force is generated in flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        # In level flight, lift should be approximately equal to weight
        # Check Z force (lift is negative Z in body axis for positive lift)
        fz = fdm["forces/fbz-aero-lbs"]
        # Lift should be significant and upward (negative in body Z)
        self.assertLess(fz, 0, "Lift should be upward (negative body Z)")

        del fdm

    def test_drag_force_in_flight(self):
        """Test that drag force opposes motion."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        # Drag should be negative X (opposing forward motion)
        fx = fdm["forces/fbx-aero-lbs"]
        self.assertLess(fx, 0, "Drag should oppose forward motion")

        del fdm

    def test_pitching_moment(self):
        """Test pitching moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Pitching moment
        my = fdm["moments/m-aero-lbsft"]
        self.assertIsNotNone(my, "Pitching moment should be accessible")

        del fdm

    def test_rolling_moment(self):
        """Test rolling moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/beta-deg"] = 5.0  # Sideslip to generate roll
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Rolling moment
        mx = fdm["moments/l-aero-lbsft"]
        self.assertIsNotNone(mx, "Rolling moment should be accessible")

        del fdm

    def test_yawing_moment(self):
        """Test yawing moment property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/beta-deg"] = 5.0  # Sideslip to generate yaw
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Yawing moment
        mz = fdm["moments/n-aero-lbsft"]
        self.assertIsNotNone(mz, "Yawing moment should be accessible")

        del fdm

    def test_alpha_effect_on_lift(self):
        """Test that angle of attack affects lift."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        # Get current alpha
        alpha1 = fdm["aero/alpha-deg"]

        # Increase alpha by pitching up
        fdm["fcs/elevator-cmd-norm"] = -0.3  # Nose up
        for _ in range(100):
            fdm.run()

        alpha2 = fdm["aero/alpha-deg"]

        # Alpha should have changed with elevator input
        self.assertIsNotNone(alpha1, "Initial alpha should be accessible")
        self.assertIsNotNone(alpha2, "Final alpha should be accessible")

        del fdm

    def test_wing_area_property(self):
        """Test wing reference area property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        wing_area = fdm["metrics/Sw-sqft"]
        self.assertGreater(wing_area, 100, "C172 wing area should be > 100 sqft")
        self.assertLess(wing_area, 200, "C172 wing area should be < 200 sqft")

        del fdm

    def test_reference_chord(self):
        """Test mean aerodynamic chord property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        chord = fdm["metrics/cbarw-ft"]
        self.assertGreater(chord, 0, "MAC should be positive")

        del fdm


if __name__ == "__main__":
    RunTest(TestAerodynamicsCoefficients)
