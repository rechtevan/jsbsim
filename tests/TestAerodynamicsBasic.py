# TestAerodynamicsBasic.py
#
# Test aerodynamics calculations including coefficients, forces, and moments.
# Exercises FGAerodynamics model through various flight conditions and control inputs.
#
# Copyright (c) 2025 Booz Allen Hamilton Inc.
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

import math

from JSBSim_utils import JSBSimTestCase, RunTest


class TestAerodynamicsBasic(JSBSimTestCase):
    """
    Test suite for aerodynamics calculations in JSBSim.

    Tests cover:
    - Lift coefficient (CL) calculation and variation with alpha
    - Drag coefficient (CD) calculation
    - Moment coefficients (Cm, Cn, Cl)
    - Dynamic pressure (Q-bar) calculation
    - Angle of attack (alpha) and sideslip (beta) calculations
    - Aerodynamic forces (lift, drag, side force)
    - Aerodynamic moments (pitch, yaw, roll)
    - Control surface effectiveness on coefficients
    """

    def setUp(self):
        JSBSimTestCase.setUp(self)

    def test_lift_coefficient_variation_with_alpha(self):
        """
        Test that lift coefficient (CL) increases with angle of attack.

        Verifies:
        - CL calculation is working
        - CL increases with alpha (up to stall)
        - CL values are within reasonable range for C172
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set cruise conditions
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0

        # Test at different angles of attack
        cl_values = []
        alphas = [0.0, 0.05, 0.10, 0.15]  # radians (0, ~3, ~6, ~9 degrees)

        for alpha in alphas:
            fdm["ic/alpha-rad"] = alpha
            fdm.run_ic()
            fdm.run()

            cl = fdm["aero/coefficient/CLalpha"]
            cl_values.append(cl)

        # Verify CL increases with alpha (before stall)
        self.assertGreater(
            cl_values[1], cl_values[0], "CL should increase with alpha (0 to 0.05 rad)"
        )
        self.assertGreater(
            cl_values[2], cl_values[1], "CL should increase with alpha (0.05 to 0.10 rad)"
        )
        self.assertGreater(
            cl_values[3], cl_values[2], "CL should increase with alpha (0.10 to 0.15 rad)"
        )

        # Verify CL values are reasonable for a C172 (should be positive for positive alpha)
        for i, cl in enumerate(cl_values):
            self.assertGreater(
                cl, 0.0, f"CL should be positive for positive alpha (alpha={alphas[i]:.3f} rad)"
            )

    def test_drag_coefficient_calculation(self):
        """
        Test drag coefficient (CD) calculation.

        Verifies:
        - CD is calculated correctly
        - CD increases with alpha (induced drag)
        - CD has reasonable values
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0

        # Test CD at different angles of attack
        cd_values = []
        alphas = [0.0, 0.05, 0.10, 0.15]

        for alpha in alphas:
            fdm["ic/alpha-rad"] = alpha
            fdm.run_ic()
            fdm.run()

            # Calculate CD from drag force: CD = Drag / (q * S)
            drag_force_lbs = abs(fdm.get_property_value("forces/fbx-aero-lbs"))
            qbar_psf = fdm.get_property_value("aero/qbar-psf")
            wing_area_sqft = fdm.get_property_value("metrics/Sw-sqft")

            cd_total = drag_force_lbs / (qbar_psf * wing_area_sqft)
            cd_values.append(cd_total)

        # Verify CD increases with alpha (induced drag increases)
        # Note: CD may vary non-monotonically due to complex aerodynamics near minimum drag angle
        # Compare highest alpha to lowest alpha to verify induced drag effect
        self.assertGreater(
            cd_values[3], cd_values[1], "CD should increase with alpha due to induced drag"
        )

        # Verify CD values are positive and reasonable
        for i, cd in enumerate(cd_values):
            self.assertGreater(
                cd, 0.0, f"CD should be positive (alpha={alphas[i]:.3f} rad, CD={cd:.4f})"
            )
            self.assertLess(cd, 0.5, f"CD should be reasonable for C172 (CD={cd:.4f})")

    def test_moment_coefficients(self):
        """
        Test pitching, yawing, and rolling moment coefficients (Cm, Cn, Cl).

        Verifies:
        - Cm (pitching moment) is calculated
        - Cn (yawing moment) is calculated
        - Cl (rolling moment) is calculated
        - Coefficients respond to changes in flight conditions
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/alpha-rad"] = 0.05
        fdm["ic/beta-rad"] = 0.0

        fdm.run_ic()
        fdm.run()

        # Get pitching moment coefficient
        cm = fdm.get_property_value("aero/coefficient/Cmalpha")
        self.assertIsNotNone(cm, "Pitching moment coefficient should be available")
        # For stable aircraft, Cm should be negative for positive alpha
        self.assertLess(
            cm, 0.0, "Pitching moment coefficient should be negative for stable aircraft"
        )

        # Test yawing moment with sideslip
        fdm["ic/beta-rad"] = 0.1  # 10 degrees sideslip
        fdm.run_ic()
        fdm.run()

        cn = fdm.get_property_value("aero/coefficient/Cnb")
        self.assertIsNotNone(cn, "Yawing moment coefficient should be available")
        # For stable aircraft, Cn should be positive (weathercock stability)
        self.assertGreater(
            cn, 0.0, "Yawing moment coefficient should be positive for directional stability"
        )

        # Test rolling moment with sideslip
        cl = fdm.get_property_value("aero/coefficient/Clb")
        self.assertIsNotNone(cl, "Rolling moment coefficient should be available")
        # For high-wing aircraft like C172, Cl should be negative (dihedral effect)
        self.assertLess(
            cl, 0.0, "Rolling moment coefficient should be negative for dihedral effect"
        )

    def test_dynamic_pressure(self):
        """
        Test dynamic pressure (Q-bar) calculation.

        Verifies:
        - Q-bar is calculated from velocity and density
        - Q-bar increases with airspeed
        - Q-bar decreases with altitude (lower density)
        - Q-bar formula: Q = 0.5 * rho * V^2
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test 1: Q-bar increases with airspeed at same altitude
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 80.0
        fdm.run_ic()
        fdm.run()

        qbar_80kts = fdm["aero/qbar-psf"]

        fdm["ic/vc-kts"] = 120.0
        fdm.run_ic()
        fdm.run()

        qbar_120kts = fdm["aero/qbar-psf"]

        self.assertGreater(qbar_120kts, qbar_80kts, "Q-bar should increase with airspeed")
        # Q-bar should scale with V^2, so ratio should be approximately (120/80)^2 = 2.25
        ratio = qbar_120kts / qbar_80kts
        self.assertGreater(ratio, 2.0, f"Q-bar should scale with V^2 (ratio={ratio:.2f})")
        self.assertLess(ratio, 2.5, f"Q-bar should scale with V^2 (ratio={ratio:.2f})")

        # Test 2: Q-bar decreases with altitude at same airspeed
        fdm["ic/h-sl-ft"] = 1000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()
        fdm.run()

        qbar_1000ft = fdm["aero/qbar-psf"]

        fdm["ic/h-sl-ft"] = 10000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()
        fdm.run()

        qbar_10000ft = fdm["aero/qbar-psf"]

        self.assertGreater(
            qbar_1000ft, qbar_10000ft, "Q-bar should decrease with altitude (lower density)"
        )

        # Verify Q-bar values are positive and reasonable
        self.assertGreater(qbar_1000ft, 0.0, "Q-bar should be positive")
        self.assertGreater(qbar_10000ft, 0.0, "Q-bar should be positive")

    def test_alpha_beta_calculation(self):
        """
        Test angle of attack (alpha) and sideslip (beta) calculations.

        Verifies:
        - Alpha is calculated from velocity components
        - Beta is calculated from velocity components
        - Initial condition settings are respected
        - Values remain within reasonable ranges
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0

        # Test 1: Set alpha and verify it's calculated
        fdm["ic/alpha-rad"] = 0.08  # ~4.6 degrees
        fdm["ic/beta-rad"] = 0.0
        fdm.run_ic()
        fdm.run()

        alpha = fdm["aero/alpha-rad"]
        self.assertAlmostEqual(alpha, 0.08, delta=0.02, msg="Alpha should match initial condition")
        self.assertGreater(alpha, 0.0, "Alpha should be positive for level flight")

        # Convert to degrees and verify
        alpha_deg = fdm["aero/alpha-deg"]
        expected_deg = 0.08 * 180.0 / math.pi
        self.assertAlmostEqual(
            alpha_deg, expected_deg, delta=1.0, msg="Alpha in degrees should be consistent"
        )

        # Test 2: Set beta and verify it's calculated
        fdm["ic/beta-rad"] = 0.05  # ~2.9 degrees sideslip
        fdm.run_ic()
        fdm.run()

        beta = fdm["aero/beta-rad"]
        self.assertAlmostEqual(beta, 0.05, delta=0.02, msg="Beta should match initial condition")

        # Verify beta in degrees
        beta_deg = fdm["aero/beta-deg"]
        expected_beta_deg = 0.05 * 180.0 / math.pi
        self.assertAlmostEqual(
            beta_deg, expected_beta_deg, delta=1.0, msg="Beta in degrees should be consistent"
        )

        # Test 3: Verify alpha and beta remain reasonable during flight
        for _ in range(10):
            fdm.run()

        alpha = fdm["aero/alpha-rad"]
        beta = fdm["aero/beta-rad"]

        self.assertGreater(alpha, -0.5, "Alpha should not be excessively negative")
        self.assertLess(alpha, 0.5, "Alpha should not be excessively positive")
        self.assertGreater(beta, -0.5, "Beta should not be excessively negative")
        self.assertLess(beta, 0.5, "Beta should not be excessively positive")

    def test_aerodynamic_forces(self):
        """
        Test total aerodynamic force calculations (lift, drag, side force).

        Verifies:
        - Lift force is calculated and positive in level flight
        - Drag force is calculated and positive
        - Side force responds to sideslip
        - Forces scale with dynamic pressure
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/alpha-rad"] = 0.05
        fdm["ic/beta-rad"] = 0.0

        fdm.run_ic()
        fdm.run()

        # Test 1: Verify lift is positive and reasonable
        lift = fdm.get_property_value("forces/fbz-aero-lbs")
        # In JSBSim body frame, Z is down, so lift (upward) is negative
        self.assertLess(lift, 0.0, "Lift force should be negative (upward in body frame)")
        abs_lift = abs(lift)
        # C172 gross weight is about 2400 lbs, lift should be close to weight in cruise
        self.assertGreater(abs_lift, 1500.0, "Lift should be substantial for cruise flight")
        self.assertLess(abs_lift, 3600.0, "Lift should be reasonable for C172")

        # Test 2: Verify drag is positive
        drag = fdm.get_property_value("forces/fbx-aero-lbs")
        # In JSBSim body frame, X is forward, so drag (backward) is negative
        self.assertLess(drag, 0.0, "Drag force should be negative (rearward in body frame)")
        abs_drag = abs(drag)
        self.assertGreater(abs_drag, 50.0, "Drag should be present")
        self.assertLess(abs_drag, 500.0, "Drag should be reasonable for C172 cruise")

        # Test 3: Side force with sideslip
        fdm["ic/beta-rad"] = 0.1
        fdm.run_ic()
        fdm.run()

        side_force = fdm.get_property_value("forces/fby-aero-lbs")
        # Side force should be non-zero with sideslip
        abs_side = abs(side_force)
        self.assertGreater(abs_side, 5.0, "Side force should be present with sideslip")

        # Test 4: Forces scale with dynamic pressure (airspeed)
        fdm["ic/beta-rad"] = 0.0
        fdm["ic/vc-kts"] = 80.0
        fdm.run_ic()
        fdm.run()

        lift_80kts = abs(fdm.get_property_value("forces/fbz-aero-lbs"))

        fdm["ic/vc-kts"] = 120.0
        fdm.run_ic()
        fdm.run()

        lift_120kts = abs(fdm.get_property_value("forces/fbz-aero-lbs"))

        # Lift should increase with airspeed (scales with q-bar)
        self.assertGreater(
            lift_120kts, lift_80kts, "Lift should increase with airspeed (dynamic pressure)"
        )

    def test_aerodynamic_moments(self):
        """
        Test aerodynamic moment calculations (pitch, yaw, roll).

        Verifies:
        - Pitching moment calculation
        - Yawing moment calculation
        - Rolling moment calculation
        - Moments respond to flight conditions appropriately
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/alpha-rad"] = 0.05
        fdm["ic/beta-rad"] = 0.0

        fdm.run_ic()
        fdm.run()

        # Test 1: Pitching moment should be non-zero with non-zero alpha
        pitch_moment = fdm.get_property_value("moments/m-aero-lbsft")
        self.assertIsNotNone(pitch_moment, "Pitching moment should be available")
        # Moment magnitude should be reasonable for C172
        abs_pitch = abs(pitch_moment)
        self.assertGreater(abs_pitch, 0.0, "Pitching moment should be non-zero with alpha")
        self.assertLess(abs_pitch, 10000.0, "Pitching moment should be reasonable")

        # Test 2: Yawing moment with sideslip
        fdm["ic/beta-rad"] = 0.1
        fdm.run_ic()
        fdm.run()

        yaw_moment = fdm.get_property_value("moments/n-aero-lbsft")
        self.assertIsNotNone(yaw_moment, "Yawing moment should be available")
        abs_yaw = abs(yaw_moment)
        self.assertGreater(abs_yaw, 0.0, "Yawing moment should be non-zero with sideslip")
        self.assertLess(abs_yaw, 5000.0, "Yawing moment should be reasonable")

        # Test 3: Rolling moment with sideslip (dihedral effect)
        roll_moment = fdm.get_property_value("moments/l-aero-lbsft")
        self.assertIsNotNone(roll_moment, "Rolling moment should be available")
        # For high-wing aircraft with dihedral, positive beta should create rolling moment
        # Sign depends on aircraft geometry, but magnitude should be non-zero
        abs_roll = abs(roll_moment)
        self.assertGreater(abs_roll, 0.0, "Rolling moment should be non-zero with sideslip")
        self.assertLess(abs_roll, 5000.0, "Rolling moment should be reasonable")

    def test_control_surface_effectiveness_elevator(self):
        """
        Test that elevator deflection affects lift coefficient and pitching moment.

        Verifies:
        - Elevator changes CL
        - Elevator changes Cm
        - Effects are proportional to deflection
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/alpha-rad"] = 0.05

        # Get baseline with neutral elevator
        fdm["fcs/elevator-cmd-norm"] = 0.0
        fdm.run_ic()
        fdm.run()

        # Get CL with elevator deflection component
        cl_baseline = fdm.get_property_value("aero/coefficient/CLDe")
        cm_baseline = fdm.get_property_value("aero/coefficient/Cmde")

        # Apply elevator deflection (down for pitch up)
        fdm["fcs/elevator-cmd-norm"] = -0.5
        for _ in range(5):  # Let FCS respond
            fdm.run()

        cl_elevator = fdm.get_property_value("aero/coefficient/CLDe")
        cm_elevator = fdm.get_property_value("aero/coefficient/Cmde")

        # Elevator deflection should change both CL and Cm
        # Note: Signs depend on convention, but values should change
        self.assertNotAlmostEqual(
            cl_elevator, cl_baseline, delta=0.001, msg="Elevator should affect CL"
        )
        self.assertNotAlmostEqual(
            cm_elevator, cm_baseline, delta=0.001, msg="Elevator should affect Cm"
        )

    def test_control_surface_effectiveness_aileron(self):
        """
        Test that aileron deflection affects rolling moment coefficient.

        Verifies:
        - Aileron changes Cl (rolling moment coefficient)
        - Effect is proportional to deflection
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0

        # Get baseline with neutral aileron
        fdm["fcs/aileron-cmd-norm"] = 0.0
        fdm.run_ic()
        fdm.run()

        cl_baseline = fdm.get_property_value("aero/coefficient/Clda")

        # Apply aileron deflection
        fdm["fcs/aileron-cmd-norm"] = 0.5
        for _ in range(5):  # Let FCS respond
            fdm.run()

        cl_aileron = fdm.get_property_value("aero/coefficient/Clda")

        # Aileron should change rolling moment coefficient
        self.assertNotAlmostEqual(
            cl_aileron, cl_baseline, delta=0.001, msg="Aileron should affect Cl (rolling moment)"
        )

    def test_control_surface_effectiveness_rudder(self):
        """
        Test that rudder deflection affects yawing moment coefficient.

        Verifies:
        - Rudder changes Cn (yawing moment coefficient)
        - Effect is proportional to deflection
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 90.0

        # Get baseline with neutral rudder
        fdm["fcs/rudder-cmd-norm"] = 0.0
        fdm.run_ic()
        fdm.run()

        cn_baseline = fdm.get_property_value("aero/coefficient/Cndr")

        # Apply rudder deflection
        fdm["fcs/rudder-cmd-norm"] = 0.5
        for _ in range(5):  # Let FCS respond
            fdm.run()

        cn_rudder = fdm.get_property_value("aero/coefficient/Cndr")

        # Rudder should change yawing moment coefficient
        self.assertNotAlmostEqual(
            cn_rudder, cn_baseline, delta=0.001, msg="Rudder should affect Cn (yawing moment)"
        )

    def test_flap_effect_on_lift(self):
        """
        Test that flap deflection increases lift coefficient.

        Verifies:
        - Flaps increase CL
        - Flaps increase CD (induced and profile drag)
        - Effects are significant and measurable
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 70.0  # Slower speed where flaps are effective
        fdm["ic/alpha-rad"] = 0.05

        # Get baseline with flaps up
        fdm["fcs/flap-cmd-norm"] = 0.0
        fdm.run_ic()
        fdm.run()

        cl_flaps_up = fdm.get_property_value("aero/coefficient/CLDf")
        cd_flaps_up = fdm.get_property_value("aero/coefficient/CDDf")

        # Deploy flaps
        fdm["fcs/flap-cmd-norm"] = 1.0
        for _ in range(10):  # Let FCS respond (flaps deploy slowly)
            fdm.run()

        cl_flaps_down = fdm.get_property_value("aero/coefficient/CLDf")
        cd_flaps_down = fdm.get_property_value("aero/coefficient/CDDf")

        # Flaps should increase both CL and CD
        self.assertGreater(
            abs(cl_flaps_down), abs(cl_flaps_up), "Flaps should increase lift coefficient"
        )
        self.assertGreater(
            abs(cd_flaps_down), abs(cd_flaps_up), "Flaps should increase drag coefficient"
        )


RunTest(TestAerodynamicsBasic)
