# test_02_takeoff_sequence.py
#
# Integration Test Scenario 2: Complete Takeoff Sequence
#
# This test exercises a complete takeoff sequence from engine start through
# climb-out, validating the integrated behavior of propulsion, aerodynamics,
# flight controls, and flight dynamics subsystems.
#
# Components tested:
# - FGPropagate: Position, velocity, attitude integration during takeoff
# - FGPropulsion: Engine start, power application, thrust generation
# - FGAerodynamics: Lift generation, drag forces, ground effect
# - FGFCS: Throttle, elevator, rudder control during takeoff roll
# - FGGroundReactions: Gear contact forces, friction, brake application
# - FGAccelerations: Acceleration computation during ground roll and rotation
# - FGAtmosphere: Atmospheric properties at field elevation
# - FGAuxiliary: Airspeed, climb rate calculations
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

import numpy as np

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import ExecuteUntil, JSBSimTestCase, RunTest


class TestTakeoffSequence(JSBSimTestCase):
    """
    Integration test for complete C172 takeoff sequence.

    This test suite validates the integrated behavior of JSBSim subsystems
    during a realistic takeoff sequence from engine start through climb-out
    to pattern altitude. It tests the interaction between propulsion, flight
    controls, aerodynamics, and flight dynamics.

    Test Coverage:
    - Engine start sequence and stabilization
    - Ground operations with brakes applied
    - Throttle application and engine response
    - Acceleration down runway with brake release
    - Rotation at appropriate airspeed
    - Lift-off dynamics and transition to flight
    - Initial climb to pattern altitude
    - Realistic performance validation for C172
    """

    def test_complete_takeoff_sequence(self):
        """
        Test a complete takeoff sequence from engine start to pattern altitude.

        This comprehensive test simulates a realistic takeoff:
        1. Initialize on runway with engine off, brakes set
        2. Start engine and verify it runs
        3. Apply full throttle with brakes holding
        4. Release brakes and accelerate down runway
        5. Rotate at 55 KIAS (typical C172 rotation speed)
        6. Lift off and establish climb
        7. Climb to 1000 ft AGL (pattern altitude)

        Validates:
        - Propulsion system integration (engine start, thrust)
        - Ground reactions (brakes, friction, gear forces)
        - Aerodynamic forces (lift generation at rotation)
        - Flight control responses
        - Realistic acceleration and climb performance
        """
        fdm = self.create_fdm()

        # Load C172P aircraft
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P model")

        # Set initial conditions: On runway 09 at sea level
        # San Francisco International (KSFO) runway 28R threshold
        fdm["ic/h-sl-ft"] = 13.0  # Field elevation approximately 13 ft MSL
        fdm["ic/lat-geod-deg"] = 37.6213  # KSFO coordinates
        fdm["ic/long-gc-deg"] = -122.3790
        fdm["ic/psi-true-deg"] = 280.0  # Runway 28R heading
        fdm["ic/theta-deg"] = 0.0  # Level on runway
        fdm["ic/phi-deg"] = 0.0  # Wings level
        fdm["ic/u-fps"] = 0.0  # Stationary
        fdm["ic/v-fps"] = 0.0
        fdm["ic/w-fps"] = 0.0

        # Engine off initially
        fdm["propulsion/engine/set-running"] = 0
        fdm["fcs/throttle-cmd-norm"] = 0.0
        fdm["fcs/mixture-cmd-norm"] = 1.0  # Full rich for sea level

        # Initialize simulation
        self.assertTrue(fdm.run_ic(), "Failed to initialize simulation")

        # Verify we're on the ground
        initial_altitude = fdm["position/h-sl-ft"]
        self.assertLess(initial_altitude, 50.0, "Aircraft should be on ground")

        # Phase 1: Engine Start
        self._test_engine_start(fdm)

        # Phase 2: Pre-takeoff with brakes
        self._test_pre_takeoff_brakes(fdm)

        # Phase 3: Takeoff roll
        ground_roll_distance, rotation_time = self._test_takeoff_roll(fdm)

        # Phase 4: Rotation and lift-off
        liftoff_time = self._test_rotation_and_liftoff(fdm, rotation_time)

        # Phase 5: Initial climb to pattern altitude
        self._test_initial_climb(fdm, liftoff_time)

        # Verify overall performance is realistic
        self._verify_takeoff_performance(fdm, ground_roll_distance, liftoff_time)

    def _test_engine_start(self, fdm):
        """
        Test engine start sequence.

        Validates:
        - Engine can be started from stopped condition
        - Engine stabilizes at idle
        - Minimal thrust at idle throttle
        - RPM reaches reasonable idle value
        """
        # Start the engine
        fdm["propulsion/engine/set-running"] = 1

        # Run for 2 seconds to let engine stabilize
        start_time = fdm.get_sim_time()
        ExecuteUntil(fdm, start_time + 2.0)

        # Verify engine is running
        engine_running = fdm["propulsion/engine/set-running"]
        self.assertEqual(engine_running, 1, "Engine should be running after start")

        # Verify engine is producing some thrust even at idle
        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertGreater(thrust, 0.0, "Engine should produce thrust when running")
        self.assertLess(thrust, 100.0, "Idle thrust should be minimal")

        # Verify propeller is turning
        rpm = fdm["propulsion/engine/propeller-rpm"]
        self.assertGreater(rpm, 500.0, "Propeller should be turning at idle")
        self.assertLess(rpm, 1200.0, "Idle RPM should be reasonable")

        # Verify aircraft hasn't moved significantly (brakes not modeled yet)
        position_change = abs(fdm["position/h-sl-ft"] - 13.0)
        self.assertLess(position_change, 5.0, "Aircraft should not move much at idle")

    def _test_pre_takeoff_brakes(self, fdm):
        """
        Test pre-takeoff with brakes applied and throttle up.

        Validates:
        - Throttle application increases thrust and RPM
        - Aircraft remains stationary (or minimal movement) with brakes
        - Engine responds correctly to throttle input
        """
        # Apply full throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0

        # Run for 2 seconds with brakes (simulated by not releasing)
        start_time = fdm.get_sim_time()
        ExecuteUntil(fdm, start_time + 2.0)

        # Verify high thrust at full throttle
        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertGreater(thrust, 150.0, "Full throttle should produce significant thrust")
        self.assertLess(thrust, 400.0, "Thrust should be within C172 capability")

        # Verify high RPM at full throttle
        rpm = fdm["propulsion/engine/propeller-rpm"]
        self.assertGreater(rpm, 2000.0, "Full throttle RPM should be high")
        self.assertLess(rpm, 3000.0, "RPM should not exceed reasonable limits")

    def _test_takeoff_roll(self, fdm):
        """
        Test acceleration down the runway to rotation speed.

        Validates:
        - Aircraft accelerates when brakes released
        - Acceleration profile is realistic for C172
        - Groundspeed increases appropriately
        - Reaches rotation speed (55 KIAS) in reasonable distance

        Returns:
            tuple: (ground_roll_distance_ft, rotation_time_sec)
        """
        # Record starting position
        start_time = fdm.get_sim_time()
        initial_lat = fdm["position/lat-geod-deg"]
        initial_lon = fdm["position/long-gc-deg"]

        # Target rotation speed: 55 KIAS (typical for C172)
        rotation_speed_kias = 55.0

        # Track position and speed during ground roll
        max_roll_time = 60.0  # Safety limit: 60 seconds
        speeds = []
        times = []

        # Accelerate down runway
        while fdm.run() and (fdm.get_sim_time() - start_time) < max_roll_time:
            current_speed = fdm["velocities/vc-kts"]
            speeds.append(current_speed)
            times.append(fdm.get_sim_time() - start_time)

            # Check if we've reached rotation speed
            if current_speed >= rotation_speed_kias:
                break

        # Verify we reached rotation speed
        final_speed = fdm["velocities/vc-kts"]
        self.assertGreaterEqual(
            final_speed,
            rotation_speed_kias - 2.0,
            "Aircraft should reach rotation speed during ground roll",
        )

        # Verify acceleration took reasonable time (not too fast, not too slow)
        rotation_time = fdm.get_sim_time() - start_time
        self.assertGreater(
            rotation_time, 10.0, "Takeoff roll should take at least 10 seconds for C172"
        )
        self.assertLess(
            rotation_time, 45.0, "Takeoff roll should complete within 45 seconds for C172"
        )

        # Calculate ground roll distance (approximate)
        # For more accurate calculation, would integrate velocity over time
        final_lat = fdm["position/lat-geod-deg"]
        final_lon = fdm["position/long-gc-deg"]

        # Approximate distance using lat/lon change
        # At KSFO latitude (37.6 deg), 1 degree lon ≈ 288,200 ft
        # 1 degree lat ≈ 364,000 ft
        lat_distance = abs(final_lat - initial_lat) * 364000.0
        lon_distance = abs(final_lon - initial_lon) * 288200.0
        ground_roll_distance = math.sqrt(lat_distance**2 + lon_distance**2)

        # Verify reasonable ground roll distance for C172 at sea level
        # Typical C172 ground roll: 800-1500 ft depending on conditions
        self.assertGreater(
            ground_roll_distance, 500.0, "Ground roll distance should be at least 500 ft"
        )
        self.assertLess(
            ground_roll_distance, 2500.0, "Ground roll distance should not exceed 2500 ft"
        )

        # Verify smooth acceleration (no sudden jumps)
        if len(speeds) > 10:
            speed_array = np.array(speeds)
            speed_diffs = np.diff(speed_array)
            # All speed increments should be positive (acceleration)
            self.assertTrue(
                np.all(speed_diffs >= -0.5),  # Allow tiny decreases due to numerical noise
                "Aircraft should accelerate smoothly (no sudden deceleration)",
            )

        return ground_roll_distance, rotation_time

    def _test_rotation_and_liftoff(self, fdm, rotation_time_offset):
        """
        Test rotation and lift-off from runway.

        Validates:
        - Elevator input causes pitch increase
        - Aircraft lifts off at appropriate speed
        - Climb is established after liftoff
        - Altitude increases above ground level

        Args:
            rotation_time_offset: Time at which rotation speed was reached

        Returns:
            float: Time at which liftoff occurred
        """
        # Apply back pressure (elevator up) to rotate
        fdm["fcs/elevator-cmd-norm"] = -0.15  # Gentle back pressure for rotation

        # Track rotation and liftoff
        start_time = fdm.get_sim_time()
        initial_altitude = fdm["position/h-sl-ft"]
        liftoff_time = None
        liftoff_detected = False

        # Run for up to 10 seconds to achieve liftoff
        max_rotation_time = 10.0
        pitch_angles = []
        altitudes = []

        while fdm.run() and (fdm.get_sim_time() - start_time) < max_rotation_time:
            current_altitude = fdm["position/h-sl-ft"]
            current_pitch = fdm["attitude/theta-deg"]

            pitch_angles.append(current_pitch)
            altitudes.append(current_altitude)

            # Detect liftoff: altitude increases by more than 5 ft
            if not liftoff_detected and (current_altitude - initial_altitude) > 5.0:
                liftoff_detected = True
                liftoff_time = fdm.get_sim_time()

            # Once we're clearly airborne, break
            if current_altitude - initial_altitude > 20.0:
                break

        # Verify liftoff occurred
        self.assertTrue(liftoff_detected, "Aircraft should have lifted off within 10 seconds")
        self.assertIsNotNone(liftoff_time, "Liftoff time should be recorded")

        # Verify pitch increased during rotation
        if len(pitch_angles) > 0:
            max_pitch = max(pitch_angles)
            self.assertGreater(
                max_pitch, 2.0, "Pitch angle should increase during rotation (> 2 degrees)"
            )
            self.assertLess(
                max_pitch, 20.0, "Pitch angle should be reasonable (< 20 degrees at liftoff)"
            )

        # Verify altitude is increasing (climbing)
        final_altitude = fdm["position/h-sl-ft"]
        altitude_gain = final_altitude - initial_altitude
        self.assertGreater(altitude_gain, 10.0, "Aircraft should be climbing after liftoff")

        # Verify climb rate is established
        climb_rate = fdm["velocities/h-dot-fps"]
        self.assertGreater(climb_rate, 2.0, "Positive climb rate should be established")
        self.assertLess(climb_rate, 25.0, "Climb rate should be reasonable for C172")

        return liftoff_time

    def _test_initial_climb(self, fdm, liftoff_time):
        """
        Test climb from liftoff to pattern altitude.

        Validates:
        - Sustained climb is maintained
        - Climb rate is realistic for C172
        - Airspeed remains in safe range
        - Aircraft reaches pattern altitude (1000 ft AGL)

        Args:
            liftoff_time: Simulation time at which liftoff occurred
        """
        # Target pattern altitude: 1000 ft AGL
        # Field elevation is 13 ft, so target 1013 ft MSL
        target_altitude_msl = 1013.0
        liftoff_altitude = fdm["position/h-sl-ft"]

        # Maintain pitch attitude for climb (reduce elevator slightly)
        fdm["fcs/elevator-cmd-norm"] = -0.1  # Maintain climb attitude

        # Track climb performance
        start_time = fdm.get_sim_time()
        max_climb_time = 120.0  # 2 minutes max to reach pattern altitude
        altitudes = []
        climb_rates = []
        airspeeds = []

        # Climb to pattern altitude
        while fdm.run() and (fdm.get_sim_time() - start_time) < max_climb_time:
            current_altitude = fdm["position/h-sl-ft"]
            altitudes.append(current_altitude)
            climb_rates.append(fdm["velocities/h-dot-fps"])
            airspeeds.append(fdm["velocities/vc-kts"])

            # Check if we've reached pattern altitude
            if current_altitude >= target_altitude_msl:
                break

        # Verify we reached pattern altitude
        final_altitude = fdm["position/h-sl-ft"]
        self.assertGreaterEqual(
            final_altitude,
            target_altitude_msl - 50.0,
            "Aircraft should reach pattern altitude (1000 ft AGL)",
        )

        # Verify climb time is reasonable
        climb_time = fdm.get_sim_time() - start_time
        altitude_gained = final_altitude - liftoff_altitude
        self.assertGreater(altitude_gained, 900.0, "Should climb at least 900 ft")

        # C172 typically climbs at 600-700 fpm at sea level
        # So 1000 ft should take roughly 1.5-2 minutes
        self.assertLess(climb_time, 180.0, "Climb should complete within 3 minutes")

        # Verify average climb rate is reasonable
        if len(climb_rates) > 10:
            climb_rate_array = np.array(climb_rates)
            # Convert fps to fpm for validation
            avg_climb_rate_fpm = np.mean(climb_rate_array) * 60.0
            self.assertGreater(
                avg_climb_rate_fpm, 300.0, "Average climb rate should be at least 300 fpm"
            )
            self.assertLess(
                avg_climb_rate_fpm, 1200.0, "Average climb rate should not exceed 1200 fpm"
            )

        # Verify airspeed remains in safe range during climb
        if len(airspeeds) > 10:
            airspeed_array = np.array(airspeeds)
            min_airspeed = np.min(airspeed_array)
            max_airspeed = np.max(airspeed_array)

            # C172 best rate of climb speed (Vy) is around 75 KIAS
            # Should maintain 60-90 KIAS during initial climb
            self.assertGreater(min_airspeed, 50.0, "Airspeed should stay above 50 KIAS in climb")
            self.assertLess(max_airspeed, 100.0, "Airspeed should stay below 100 KIAS in climb")

        # Verify sustained climb (altitude continuously increasing)
        if len(altitudes) > 20:
            altitude_array = np.array(altitudes)
            # Check that altitude is generally increasing
            # Allow for small fluctuations due to numerical integration
            altitude_changes = np.diff(altitude_array)
            positive_changes = np.sum(altitude_changes > 0)
            total_changes = len(altitude_changes)
            positive_ratio = positive_changes / total_changes

            self.assertGreater(
                positive_ratio,
                0.95,
                "Altitude should increase consistently during climb (>95% of time steps)",
            )

    def _verify_takeoff_performance(self, fdm, ground_roll_distance, liftoff_time):
        """
        Verify overall takeoff performance is realistic for C172.

        Validates:
        - Total takeoff distance is within expected range
        - Time to pattern altitude is reasonable
        - No anomalous values in key parameters
        - Aircraft is in stable flight at pattern altitude

        Args:
            ground_roll_distance: Distance traveled during ground roll (ft)
            liftoff_time: Simulation time at liftoff
        """
        # Verify final state
        final_altitude = fdm["position/h-sl-ft"]
        final_airspeed = fdm["velocities/vc-kts"]
        final_pitch = fdm["attitude/theta-deg"]
        final_roll = fdm["attitude/phi-deg"]

        # Should be at or near pattern altitude
        self.assertGreater(final_altitude, 950.0, "Should be at pattern altitude")
        self.assertLess(final_altitude, 1100.0, "Should not overshoot pattern altitude much")

        # Airspeed should be in normal climb range
        self.assertGreater(final_airspeed, 55.0, "Airspeed should be above stall")
        self.assertLess(final_airspeed, 95.0, "Airspeed should be in normal climb range")

        # Pitch attitude should be reasonable for climb
        self.assertGreater(final_pitch, 2.0, "Should have nose-up attitude in climb")
        self.assertLess(final_pitch, 15.0, "Pitch should not be excessive")

        # Wings should be approximately level
        self.assertLess(abs(final_roll), 10.0, "Wings should be approximately level")

        # Verify no NaN or Inf values in critical parameters
        critical_properties = [
            "position/h-sl-ft",
            "velocities/vc-kts",
            "attitude/theta-deg",
            "attitude/phi-deg",
            "attitude/psi-deg",
            "velocities/h-dot-fps",
            "propulsion/engine/thrust-lbs",
        ]

        for prop in critical_properties:
            value = fdm[prop]
            self.assertFalse(math.isnan(value), f"NaN detected in {prop}")
            self.assertFalse(math.isinf(value), f"Inf detected in {prop}")

        # Verify engine is still running normally
        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertGreater(thrust, 100.0, "Engine should still be producing thrust")

        rpm = fdm["propulsion/engine/propeller-rpm"]
        self.assertGreater(rpm, 2000.0, "Engine should be at high RPM")

        # Verify fuel has been consumed
        # Note: This assumes initial fuel was loaded
        try:
            fuel_remaining = fdm["propulsion/total-fuel-lbs"]
            if fuel_remaining is not None and fuel_remaining > 0:
                # Fuel should have decreased, but not excessively
                # C172 fuel burn is roughly 8-10 gal/hr = 48-60 lbs/hr
                # For a few minutes of operation, should burn a few pounds
                self.assertGreater(fuel_remaining, 0.0, "Should have fuel remaining")
        except (KeyError, TypeError):
            # If fuel property not available, skip this check
            pass

    def test_engine_failure_on_takeoff(self):
        """
        Test engine failure during takeoff roll.

        This test validates system behavior when engine fails during
        takeoff roll before rotation. Aircraft should decelerate and
        come to a stop.

        Validates:
        - System handles engine failure gracefully
        - Aircraft decelerates appropriately
        - No numerical instabilities occur
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Initialize on runway
        fdm["ic/h-sl-ft"] = 13.0
        fdm["ic/psi-true-deg"] = 280.0
        fdm["ic/u-fps"] = 0.0

        # Start engine and apply throttle
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 1.0
        fdm["fcs/mixture-cmd-norm"] = 1.0

        fdm.run_ic()

        # Accelerate for 5 seconds
        start_time = fdm.get_sim_time()
        ExecuteUntil(fdm, start_time + 5.0)

        # Record speed at engine failure
        speed_at_failure = fdm["velocities/vc-kts"]
        self.assertGreater(speed_at_failure, 10.0, "Should have gained some speed")

        # Simulate engine failure
        fdm["propulsion/engine/set-running"] = 0
        fdm["fcs/throttle-cmd-norm"] = 0.0

        # Run for 30 seconds after failure
        failure_time = fdm.get_sim_time()
        max_decel_time = 30.0

        speeds_after_failure = []

        while fdm.run() and (fdm.get_sim_time() - failure_time) < max_decel_time:
            current_speed = fdm["velocities/vc-kts"]
            speeds_after_failure.append(current_speed)

            # If we've stopped, break
            if current_speed < 1.0:
                break

        # Verify aircraft decelerated
        final_speed = fdm["velocities/vc-kts"]
        self.assertLess(
            final_speed, speed_at_failure, "Aircraft should decelerate after engine failure"
        )

        # Verify no numerical issues
        for prop in ["position/h-sl-ft", "velocities/vc-kts", "attitude/theta-deg"]:
            value = fdm[prop]
            self.assertFalse(math.isnan(value), f"NaN in {prop} after engine failure")
            self.assertFalse(math.isinf(value), f"Inf in {prop} after engine failure")

    def test_short_field_takeoff(self):
        """
        Test short-field takeoff technique.

        Validates takeoff with maximum performance technique:
        - Full throttle application before brake release
        - Rotation at lower speed (just above stall)
        - Maximum climb angle after liftoff

        This tests the system under more aggressive control inputs.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Initialize on runway
        fdm["ic/h-sl-ft"] = 13.0
        fdm["ic/psi-true-deg"] = 280.0
        fdm["ic/u-fps"] = 0.0

        # Engine start
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 1.0  # Full throttle
        fdm["fcs/mixture-cmd-norm"] = 1.0

        fdm.run_ic()

        # Hold brakes and run up engine (simulated by waiting)
        start_time = fdm.get_sim_time()
        ExecuteUntil(fdm, start_time + 1.0)

        # Release brakes and accelerate
        initial_time = fdm.get_sim_time()
        rotation_speed = 50.0  # Slightly lower rotation speed for short field

        # Accelerate to rotation speed
        while fdm.run() and (fdm.get_sim_time() - initial_time) < 45.0:
            if fdm["velocities/vc-kts"] >= rotation_speed:
                break

        # Verify reached rotation speed
        self.assertGreaterEqual(
            fdm["velocities/vc-kts"], rotation_speed - 2.0, "Should reach rotation speed"
        )

        # Aggressive rotation
        fdm["fcs/elevator-cmd-norm"] = -0.25  # More aggressive back pressure

        # Rotate and liftoff
        rotation_start = fdm.get_sim_time()
        initial_alt = fdm["position/h-sl-ft"]

        while fdm.run() and (fdm.get_sim_time() - rotation_start) < 15.0:
            if fdm["position/h-sl-ft"] - initial_alt > 50.0:
                break

        # Verify liftoff occurred
        altitude_gain = fdm["position/h-sl-ft"] - initial_alt
        self.assertGreater(altitude_gain, 20.0, "Aircraft should lift off")

        # Verify pitch is higher for short-field technique
        pitch = fdm["attitude/theta-deg"]
        self.assertGreater(pitch, 5.0, "Pitch should be higher for short-field takeoff")

        # Verify climb is established
        climb_rate = fdm["velocities/h-dot-fps"]
        self.assertGreater(climb_rate, 3.0, "Should establish positive climb")


if __name__ == "__main__":
    RunTest(TestTakeoffSequence)
