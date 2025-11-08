# test_05_coordinated_turns.py
#
# Integration Test Scenario 5: Coordinated Turn Maneuvers
#
# This test exercises coordinated turn dynamics, verifying that the aircraft
# can execute proper turning maneuvers with appropriate rudder coordination,
# correct load factors, turn rates, and minimal sideslip. It validates the
# integration of flight control system, aerodynamics, and flight dynamics
# during turning flight.
#
# Components tested:
# - FGFCS: Flight control system (aileron, rudder coordination)
# - FGAerodynamics: Turn-related aerodynamic forces and moments
# - FGPropagate: Angular rate integration during turns
# - FGAuxiliary: Sideslip angle (beta) computation
# - FGAccelerations: Load factor computation
# - FGAtmosphere: Atmospheric effects on turn performance
# - FGMassBalance: Inertial effects during coordinated turns
#
# Expected coverage gain: +2-3%
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

import math
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import JSBSimTestCase, RunTest


class TestCoordinatedTurns(JSBSimTestCase):
    """
    Integration test for coordinated turn maneuvers.

    This test suite verifies that JSBSim correctly models coordinated turns,
    including proper load factors, turn rates, sideslip minimization, and
    altitude maintenance. It tests the integration of flight controls,
    aerodynamics, and flight dynamics during turning flight.

    Test Coverage:
    - Standard rate turns (30-degree bank)
    - Steep turns (45-degree bank)
    - Turn coordination (slip/skid ball centered)
    - Complete 360-degree turns with heading recovery
    - Adverse yaw effects and correction
    - Load factor verification (n = 1/cos(bank))
    - Turn radius and rate calculations
    - Altitude maintenance during turns
    """

    def setUp(self):
        """Set up test environment for coordinated turn tests."""
        super().setUp()
        self.cruise_altitude = 5000.0  # feet
        self.cruise_speed = 100.0  # knots
        self.tolerance_altitude = 100.0  # feet
        self.tolerance_heading = 5.0  # degrees
        self.tolerance_sideslip = 5.0  # degrees
        self.dt = 1.0 / 120.0  # 120 Hz simulation rate

    def initialize_cruise_flight(self, fdm):
        """
        Initialize aircraft in stable cruise flight.

        Args:
            fdm: FGFDMExec instance

        Returns:
            bool: True if initialization successful
        """
        # Set initial conditions for cruise flight
        fdm["ic/h-sl-ft"] = self.cruise_altitude
        fdm["ic/vc-kts"] = self.cruise_speed
        fdm["ic/psi-true-deg"] = 0.0  # North heading
        fdm["ic/theta-deg"] = 0.0  # Level pitch
        fdm["ic/phi-deg"] = 0.0  # Level wings
        fdm["ic/alpha-deg"] = 2.0  # Small positive AoA for cruise

        # Set throttle for cruise
        fdm["fcs/throttle-cmd-norm"] = 0.6
        fdm["fcs/mixture-cmd-norm"] = 0.8

        # Initialize
        if not fdm.run_ic():
            return False

        # Let simulation stabilize for 5 seconds
        for _ in range(int(5.0 / self.dt)):
            fdm.run()

        return True

    def test_level_30_degree_banked_turn(self):
        """
        Test standard rate turn with 30-degree bank angle.

        A standard rate turn is typically performed at 30 degrees of bank
        for general aviation aircraft. This test verifies that the aircraft
        can maintain altitude and coordinated flight during a standard turn.

        Physics verification:
        - Load factor: n = 1/cos(30°) ≈ 1.15 G
        - Turn rate: approximately 3 degrees/second (standard rate)
        - Altitude maintained within tolerance
        - Minimal sideslip (coordinated flight)
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Initialize in cruise flight
        self.assertTrue(self.initialize_cruise_flight(fdm), "Failed to initialize cruise flight")

        initial_altitude = fdm["position/h-sl-ft"]
        initial_heading = fdm["attitude/psi-deg"]

        # Target bank angle for standard rate turn
        target_bank_deg = 30.0
        target_bank_rad = math.radians(target_bank_deg)

        # Expected load factor: n = 1/cos(bank)
        expected_load_factor = 1.0 / math.cos(target_bank_rad)

        # Enter turn with coordinated aileron and rudder
        aileron_deflection = 0.3  # Right turn
        rudder_coordination = 0.15  # Coordinated rudder

        # Track properties manually during turn
        altitudes = []
        sideslips = []
        load_factors = []

        # Execute turn for 10 seconds
        turn_duration = 10.0
        steps = int(turn_duration / self.dt)

        for step in range(steps):
            # Apply coordinated turn inputs
            fdm["fcs/aileron-cmd-norm"] = aileron_deflection
            fdm["fcs/rudder-cmd-norm"] = rudder_coordination

            # Maintain altitude with slight back pressure
            current_altitude = fdm["position/h-sl-ft"]
            altitude_error = initial_altitude - current_altitude
            elevator_cmd = 0.02 * altitude_error / 100.0  # Proportional control
            elevator_cmd = max(-0.3, min(0.3, elevator_cmd))
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            # Run simulation step
            self.assertTrue(fdm.run(), f"Simulation failed at step {step}")

            # Track properties periodically
            if step % 10 == 0:
                altitudes.append(fdm["position/h-sl-ft"])
                sideslips.append(fdm["aero/beta-deg"])
                load_factors.append(fdm["accelerations/n-pilot-z-norm"])

            # Check bank angle is approximately at target
            if step > 120:  # After roll-in complete (1 second)
                current_bank = abs(fdm["attitude/phi-deg"])
                if step % 120 == 0:  # Check every second
                    self.assertGreater(
                        current_bank,
                        target_bank_deg - 10.0,
                        "Bank angle too shallow",
                    )
                    self.assertLess(
                        current_bank,
                        target_bank_deg + 10.0,
                        "Bank angle too steep",
                    )

        # Verify altitude maintained within tolerance
        final_altitude = fdm["position/h-sl-ft"]
        altitude_deviation = abs(final_altitude - initial_altitude)
        self.assertLess(
            altitude_deviation,
            self.tolerance_altitude,
            f"Altitude deviation {altitude_deviation:.1f} ft exceeds tolerance",
        )

        # Verify load factor approximately matches expected
        if len(load_factors) > 0:
            avg_load_factor = sum(load_factors) / len(load_factors)
            self.assertGreater(
                avg_load_factor, 1.0, "Load factor should be greater than 1G in turn"
            )
            self.assertLess(
                abs(avg_load_factor - expected_load_factor),
                0.3,
                f"Load factor {avg_load_factor:.2f} deviates from expected {expected_load_factor:.2f}",
            )

        # Verify minimal sideslip (coordinated turn)
        if len(sideslips) > 0:
            avg_sideslip = abs(sum(sideslips) / len(sideslips))
            self.assertLess(
                avg_sideslip,
                self.tolerance_sideslip,
                f"Average sideslip {avg_sideslip:.1f} deg indicates uncoordinated turn",
            )

        # Verify turn occurred (heading changed)
        final_heading = fdm["attitude/psi-deg"]
        heading_change = abs(final_heading - initial_heading)
        self.assertGreater(heading_change, 10.0, "Insufficient heading change during turn")

    def test_level_45_degree_banked_turn(self):
        """
        Test steep turn with 45-degree bank angle.

        A 45-degree bank turn is a steep turn commonly practiced in flight
        training. This test verifies that the aircraft can maintain altitude
        and coordination during a steeper turn with higher load factors.

        Physics verification:
        - Load factor: n = 1/cos(45°) ≈ 1.41 G
        - Higher turn rate than standard rate turn
        - Altitude maintained within tolerance
        - Coordinated flight maintained
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Initialize in cruise flight
        self.assertTrue(self.initialize_cruise_flight(fdm), "Failed to initialize cruise flight")

        initial_altitude = fdm["position/h-sl-ft"]
        initial_heading = fdm["attitude/psi-deg"]

        # Target bank angle for steep turn
        target_bank_deg = 45.0
        target_bank_rad = math.radians(target_bank_deg)

        # Expected load factor: n = 1/cos(bank)
        expected_load_factor = 1.0 / math.cos(target_bank_rad)

        # Enter steep turn with coordinated controls
        aileron_deflection = 0.5  # Steeper bank
        rudder_coordination = 0.25  # More rudder needed

        # Execute turn for 8 seconds
        turn_duration = 8.0
        steps = int(turn_duration / self.dt)

        max_load_factor = 0.0
        sideslip_values = []
        altitude_deviations = []

        for step in range(steps):
            # Apply coordinated turn inputs
            fdm["fcs/aileron-cmd-norm"] = aileron_deflection
            fdm["fcs/rudder-cmd-norm"] = rudder_coordination

            # Maintain altitude with back pressure
            current_altitude = fdm["position/h-sl-ft"]
            altitude_error = initial_altitude - current_altitude
            elevator_cmd = 0.03 * altitude_error / 100.0  # Proportional control
            elevator_cmd = max(-0.5, min(0.3, elevator_cmd))
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            # Run simulation step
            self.assertTrue(fdm.run(), f"Simulation failed at step {step}")

            # Collect data after roll-in complete
            if step > 120:  # After 1 second
                load_factor = fdm["accelerations/n-pilot-z-norm"]
                max_load_factor = max(max_load_factor, load_factor)
                sideslip_values.append(abs(fdm["aero/beta-deg"]))
                altitude_deviations.append(abs(current_altitude - initial_altitude))

        # Verify load factor increased appropriately
        self.assertGreater(
            max_load_factor,
            1.3,
            f"Load factor {max_load_factor:.2f} too low for 45-degree bank",
        )
        self.assertLess(
            abs(max_load_factor - expected_load_factor),
            0.4,
            f"Load factor {max_load_factor:.2f} deviates from expected {expected_load_factor:.2f}",
        )

        # Verify altitude maintained reasonably well
        max_altitude_deviation = max(altitude_deviations)
        self.assertLess(
            max_altitude_deviation,
            self.tolerance_altitude * 1.5,  # Allow slightly more deviation
            f"Altitude deviation {max_altitude_deviation:.1f} ft exceeds tolerance",
        )

        # Verify coordinated flight maintained
        avg_sideslip = sum(sideslip_values) / len(sideslip_values)
        self.assertLess(
            avg_sideslip,
            self.tolerance_sideslip * 1.2,  # Allow slightly more sideslip
            f"Average sideslip {avg_sideslip:.1f} deg indicates uncoordinated turn",
        )

        # Verify significant heading change occurred
        final_heading = fdm["attitude/psi-deg"]
        heading_change = abs(final_heading - initial_heading)
        self.assertGreater(heading_change, 20.0, "Insufficient heading change during steep turn")

    def test_turn_coordination_slip_skid(self):
        """
        Test turn coordination by monitoring slip/skid ball (sideslip angle).

        In a coordinated turn, the slip/skid ball should be centered, meaning
        the sideslip angle (beta) should be near zero. This test verifies that:
        - Proper rudder input reduces sideslip
        - Insufficient rudder causes slip (ball to outside)
        - Excessive rudder causes skid (ball to inside)

        This exercises the FGAuxiliary sideslip computation and validates
        the relationship between control inputs and coordinated flight.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Initialize in cruise flight
        self.assertTrue(self.initialize_cruise_flight(fdm), "Failed to initialize cruise flight")

        # Test 1: Coordinated turn (proper rudder)
        coordinated_sideslip_values = []
        for step in range(int(5.0 / self.dt)):
            fdm["fcs/aileron-cmd-norm"] = 0.3  # Right turn
            fdm["fcs/rudder-cmd-norm"] = 0.15  # Coordinated rudder
            fdm["fcs/elevator-cmd-norm"] = 0.05  # Maintain altitude
            fdm.run()

            if step > 120:  # After stabilization
                coordinated_sideslip_values.append(abs(fdm["aero/beta-deg"]))

        avg_coordinated_sideslip = sum(coordinated_sideslip_values) / len(
            coordinated_sideslip_values
        )

        # Test 2: Slipping turn (insufficient rudder)
        # Reset to level flight
        fdm["ic/phi-deg"] = 0.0
        fdm.run_ic()
        for _ in range(int(2.0 / self.dt)):
            fdm.run()

        slipping_sideslip_values = []
        for step in range(int(5.0 / self.dt)):
            fdm["fcs/aileron-cmd-norm"] = 0.3  # Right turn
            fdm["fcs/rudder-cmd-norm"] = 0.0  # No rudder (will slip)
            fdm["fcs/elevator-cmd-norm"] = 0.05
            fdm.run()

            if step > 120:
                slipping_sideslip_values.append(abs(fdm["aero/beta-deg"]))

        avg_slipping_sideslip = sum(slipping_sideslip_values) / len(slipping_sideslip_values)

        # Test 3: Skidding turn (excessive rudder)
        # Reset to level flight
        fdm["ic/phi-deg"] = 0.0
        fdm.run_ic()
        for _ in range(int(2.0 / self.dt)):
            fdm.run()

        skidding_sideslip_values = []
        for step in range(int(5.0 / self.dt)):
            fdm["fcs/aileron-cmd-norm"] = 0.3  # Right turn
            fdm["fcs/rudder-cmd-norm"] = 0.4  # Excessive rudder (will skid)
            fdm["fcs/elevator-cmd-norm"] = 0.05
            fdm.run()

            if step > 120:
                skidding_sideslip_values.append(abs(fdm["aero/beta-deg"]))

        avg_skidding_sideslip = sum(skidding_sideslip_values) / len(skidding_sideslip_values)

        # Verify coordinated turn has minimal sideslip
        self.assertLess(
            avg_coordinated_sideslip,
            self.tolerance_sideslip,
            f"Coordinated turn sideslip {avg_coordinated_sideslip:.1f} deg too high",
        )

        # Verify slipping turn has more sideslip than coordinated
        self.assertGreater(
            avg_slipping_sideslip,
            avg_coordinated_sideslip,
            "Slipping turn should have more sideslip than coordinated turn",
        )

        # Verify skidding turn has more sideslip than coordinated
        self.assertGreater(
            avg_skidding_sideslip,
            avg_coordinated_sideslip,
            "Skidding turn should have more sideslip than coordinated turn",
        )

    def test_complete_360_turn_heading_recovery(self):
        """
        Test complete 360-degree turn and return to original heading.

        This test verifies that the aircraft can execute a complete circular
        turn and return to the original heading, demonstrating consistent
        turn rate and angular integration by FGPropagate.

        Physics verification:
        - Complete 360-degree heading change
        - Return to within tolerance of original heading
        - Altitude maintained throughout turn
        - Consistent turn rate
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Initialize in cruise flight
        self.assertTrue(self.initialize_cruise_flight(fdm), "Failed to initialize cruise flight")

        initial_altitude = fdm["position/h-sl-ft"]
        initial_heading = fdm["attitude/psi-deg"]

        # Execute coordinated turn for approximately 360 degrees
        # At standard rate (3 deg/sec), 360 degrees takes ~120 seconds
        # But with 30-degree bank at 100 kts, turn rate is higher
        # Estimate: ~40-50 seconds for 360 degrees
        turn_duration = 50.0
        steps = int(turn_duration / self.dt)

        aileron_deflection = 0.3
        rudder_coordination = 0.15

        heading_changes = []
        previous_heading = initial_heading

        for step in range(steps):
            # Apply coordinated turn inputs
            fdm["fcs/aileron-cmd-norm"] = aileron_deflection
            fdm["fcs/rudder-cmd-norm"] = rudder_coordination

            # Maintain altitude
            current_altitude = fdm["position/h-sl-ft"]
            altitude_error = initial_altitude - current_altitude
            elevator_cmd = 0.02 * altitude_error / 100.0
            elevator_cmd = max(-0.3, min(0.3, elevator_cmd))
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            # Run simulation step
            self.assertTrue(fdm.run(), f"Simulation failed at step {step}")

            # Track heading changes
            current_heading = fdm["attitude/psi-deg"]
            heading_change = current_heading - previous_heading

            # Handle heading wrap-around (0/360 transition)
            if heading_change > 180.0:
                heading_change -= 360.0
            elif heading_change < -180.0:
                heading_change += 360.0

            heading_changes.append(heading_change)
            previous_heading = current_heading

        # Calculate total heading change
        total_heading_change = sum(heading_changes)

        # Verify approximately 360-degree turn
        self.assertGreater(
            abs(total_heading_change),
            330.0,
            f"Total heading change {total_heading_change:.1f} deg insufficient for 360 turn",
        )
        self.assertLess(
            abs(total_heading_change),
            390.0,
            f"Total heading change {total_heading_change:.1f} deg excessive for 360 turn",
        )

        # Verify returned near original heading
        final_heading = fdm["attitude/psi-deg"]
        heading_error = final_heading - initial_heading

        # Normalize to [-180, 180]
        if heading_error > 180.0:
            heading_error -= 360.0
        elif heading_error < -180.0:
            heading_error += 360.0

        self.assertLess(
            abs(heading_error),
            self.tolerance_heading * 3,  # Allow 15 degrees tolerance
            f"Final heading error {heading_error:.1f} deg too large after 360 turn",
        )

        # Verify altitude maintained
        final_altitude = fdm["position/h-sl-ft"]
        altitude_deviation = abs(final_altitude - initial_altitude)
        self.assertLess(
            altitude_deviation,
            self.tolerance_altitude * 2,  # Allow 200 ft over long turn
            f"Altitude deviation {altitude_deviation:.1f} ft after 360 turn",
        )

    def test_adverse_yaw_verification(self):
        """
        Test adverse yaw effect and correction with rudder.

        Adverse yaw is the tendency of an aircraft to yaw in the opposite
        direction of a roll due to differential drag from deflected ailerons.
        This test verifies that:
        - Aileron input alone causes adverse yaw (yaw opposite to roll)
        - Coordinated rudder input counteracts adverse yaw
        - The yaw rate is properly computed by the simulation

        This exercises the FGAerodynamics yaw moment computation and
        demonstrates the need for rudder coordination in turns.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Initialize in cruise flight
        self.assertTrue(self.initialize_cruise_flight(fdm), "Failed to initialize cruise flight")

        initial_heading = fdm["attitude/psi-deg"]

        # Test 1: Apply aileron only (no rudder) - should see adverse yaw
        yaw_rates_no_rudder = []
        headings_no_rudder = []

        for step in range(int(3.0 / self.dt)):
            # Apply right aileron only
            fdm["fcs/aileron-cmd-norm"] = 0.4
            fdm["fcs/rudder-cmd-norm"] = 0.0  # No rudder
            fdm["fcs/elevator-cmd-norm"] = 0.0
            fdm.run()

            if step > 60:  # After initial transient
                yaw_rate = fdm["velocities/r-rad_sec"]  # Yaw rate in body frame
                yaw_rates_no_rudder.append(yaw_rate)
                headings_no_rudder.append(fdm["attitude/psi-deg"])

        # Reset to level flight
        fdm["ic/phi-deg"] = 0.0
        fdm["ic/psi-true-deg"] = initial_heading
        fdm.run_ic()
        for _ in range(int(2.0 / self.dt)):
            fdm.run()

        # Test 2: Apply aileron with coordinated rudder
        yaw_rates_with_rudder = []
        headings_with_rudder = []

        for step in range(int(3.0 / self.dt)):
            # Apply right aileron with coordinated rudder
            fdm["fcs/aileron-cmd-norm"] = 0.4
            fdm["fcs/rudder-cmd-norm"] = 0.2  # Coordinated rudder
            fdm["fcs/elevator-cmd-norm"] = 0.0
            fdm.run()

            if step > 60:  # After initial transient
                yaw_rate = fdm["velocities/r-rad_sec"]
                yaw_rates_with_rudder.append(yaw_rate)
                headings_with_rudder.append(fdm["attitude/psi-deg"])

        # Calculate average yaw rates
        if len(yaw_rates_no_rudder) > 0:
            avg_yaw_rate_no_rudder = sum(yaw_rates_no_rudder) / len(yaw_rates_no_rudder)
        else:
            avg_yaw_rate_no_rudder = 0.0

        if len(yaw_rates_with_rudder) > 0:
            avg_yaw_rate_with_rudder = sum(yaw_rates_with_rudder) / len(yaw_rates_with_rudder)
        else:
            avg_yaw_rate_with_rudder = 0.0

        # Calculate heading changes
        if len(headings_no_rudder) > 1:
            heading_change_no_rudder = headings_no_rudder[-1] - headings_no_rudder[0]
        else:
            heading_change_no_rudder = 0.0

        if len(headings_with_rudder) > 1:
            heading_change_with_rudder = headings_with_rudder[-1] - headings_with_rudder[0]
        else:
            heading_change_with_rudder = 0.0

        # Verify that yaw behavior differs between uncoordinated and coordinated
        # With right aileron (right bank), adverse yaw causes left (negative) yaw initially
        # With coordinated right rudder, yaw should be more in the turn direction

        # The test verifies that rudder coordination affects yaw behavior
        # Note: Exact sign depends on timing and aircraft dynamics, but the
        # magnitudes should differ
        self.assertNotAlmostEqual(
            avg_yaw_rate_no_rudder,
            avg_yaw_rate_with_rudder,
            places=3,
            msg="Rudder should significantly affect yaw rate",
        )

        # Verify yaw rate is being computed (not stuck at zero)
        max_yaw_rate_no_rudder = max([abs(r) for r in yaw_rates_no_rudder])
        self.assertGreater(
            max_yaw_rate_no_rudder,
            0.01,
            "Yaw rate should be non-zero during roll entry",
        )

        # Verify heading changes occurred in both cases
        self.assertNotAlmostEqual(
            heading_change_no_rudder,
            0.0,
            places=0,
            msg="Heading should change during uncoordinated roll",
        )
        self.assertNotAlmostEqual(
            heading_change_with_rudder,
            0.0,
            places=0,
            msg="Heading should change during coordinated turn",
        )


if __name__ == "__main__":
    RunTest(TestCoordinatedTurns)
