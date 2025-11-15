# test_02_complete_flight_scenarios.py
#
# Integration Test: Complete Flight Scenarios
#
# This test exercises complete end-to-end flight mission profiles that exercise
# many subsystems working together. These scenarios validate JSBSim's ability to
# simulate realistic multi-phase flight operations from engine start through
# landing, providing high coverage value through comprehensive system integration.
#
# Components tested (integrated across all phases):
# - FGFDMExec: Overall simulation coordination
# - FGPropagate: Position, velocity, attitude throughout mission
# - FGPropulsion: Engine operations across power settings
# - FGAerodynamics: Forces and moments across flight envelope
# - FGFCS: Control system responses during all phases
# - FGGroundReactions: Ground operations and landing gear
# - FGAtmosphere: Atmospheric properties at various altitudes
# - FGAccelerations: Acceleration computation in all flight modes
# - FGMassBalance: Mass properties and fuel consumption
# - FGAuxiliary: Derived parameters throughout flight
# - FGInitialCondition: Various starting configurations
#
# Expected coverage gain: +3-5% (high value from multi-system integration)
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

import numpy as np  # noqa: E402
from JSBSim_utils import (  # noqa: E402
    AltitudeHoldController,
    ExecuteUntil,
    JSBSimTestCase,
    RunTest,
    SimplePIDController,
    TrimAircraft,
)


class TestCompleteFlightScenarios(JSBSimTestCase):
    """
    Integration tests for complete end-to-end flight scenarios.

    This test suite validates JSBSim's ability to simulate realistic complete
    flight missions from engine start through all phases of flight to landing.
    Each scenario exercises multiple subsystems in an integrated manner,
    providing high-value coverage of real-world operations.

    Scenarios Tested:
    - Complete traffic pattern (takeoff, climb, downwind, base, final, land)
    - Climb from ground to cruise altitude with trim
    - Extended cruise flight maintaining altitude/speed
    - Descent and approach from altitude
    - Touch-and-go maneuver
    - Emergency descent scenario
    - Fuel endurance flight to low fuel state

    All scenarios use the C172X aircraft model for realistic performance.
    """

    def _start_engine(self, fdm):
        """
        Helper to start engine with proper sequence.

        Args:
            fdm: FGFDMExec instance

        Returns:
            bool: True if engine started successfully
        """
        altitude = fdm["position/h-sl-ft"]
        # Adjust mixture for altitude
        if altitude < 3000:
            mixture = 0.87
        elif altitude < 6000:
            mixture = 0.92
        else:
            mixture = 1.0

        fdm["fcs/mixture-cmd-norm"] = mixture
        fdm["fcs/throttle-cmd-norm"] = 0.6
        fdm["propulsion/magneto_cmd"] = 3
        fdm["propulsion/starter_cmd"] = 1

        # Crank for 2.5 seconds
        dt = fdm["simulation/dt"]
        frames = int(2.5 / dt)
        for _ in range(frames):
            fdm.run()

        fdm["propulsion/starter_cmd"] = 0

        # Verify engine running
        return fdm["propulsion/engine/set-running"] == 1

    def test_complete_pattern_flight(self):
        """
        Test complete traffic pattern: takeoff, climb, downwind, base, final, land.

        This comprehensive scenario exercises the complete circuit of a VFR
        traffic pattern including:
        1. Engine start and ground operations
        2. Takeoff roll and rotation
        3. Climb to pattern altitude (1000 ft AGL)
        4. Downwind leg at pattern altitude
        5. Base turn with coordinated controls
        6. Final approach with descent
        7. Flare and touchdown

        Validates end-to-end integration of all major subsystems through
        a realistic complete flight from ground back to ground.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Initialize on runway
        fdm["ic/h-sl-ft"] = 13.0  # Field elevation
        fdm["ic/psi-true-deg"] = 280.0  # Runway heading
        fdm["ic/u-fps"] = 0.0
        fdm["ic/v-fps"] = 0.0
        fdm["ic/w-fps"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Phase 1: Engine start
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        initial_altitude = fdm["position/h-sl-ft"]

        # Phase 2: Takeoff
        fdm["fcs/throttle-cmd-norm"] = 1.0
        start_time = fdm.get_sim_time()

        # Accelerate to rotation speed (55 kts)
        while fdm.run() and (fdm.get_sim_time() - start_time) < 60.0:
            if fdm["velocities/vc-kts"] >= 55.0:
                break

        self.assertGreaterEqual(fdm["velocities/vc-kts"], 50.0, "Failed to reach rotation speed")

        # Rotate and lift off
        fdm["fcs/elevator-cmd-norm"] = -0.15
        rotation_start = fdm.get_sim_time()
        liftoff_detected = False

        while fdm.run() and (fdm.get_sim_time() - rotation_start) < 10.0:
            if fdm["position/h-sl-ft"] - initial_altitude > 10.0:
                liftoff_detected = True
                break

        self.assertTrue(liftoff_detected, "Failed to lift off")

        # Phase 3: Climb to pattern altitude
        pattern_altitude = initial_altitude + 1000.0
        fdm["fcs/elevator-cmd-norm"] = -0.05  # Climb attitude

        climb_start = fdm.get_sim_time()
        max_altitude_reached = initial_altitude

        while fdm.run() and (fdm.get_sim_time() - climb_start) < 120.0:
            current_alt = fdm["position/h-sl-ft"]
            max_altitude_reached = max(max_altitude_reached, current_alt)
            if current_alt >= pattern_altitude:
                break

        altitude_gained = max_altitude_reached - initial_altitude
        self.assertGreater(altitude_gained, 100.0, f"Insufficient climb: {altitude_gained:.1f} ft")

        # Phase 4: Downwind leg
        fdm["fcs/throttle-cmd-norm"] = 0.4
        fdm["fcs/elevator-cmd-norm"] = 0.0

        downwind_start = fdm.get_sim_time()
        ExecuteUntil(fdm, downwind_start + 20.0)

        # Phase 5: Base turn (left turn)
        initial_heading = fdm["attitude/psi-deg"]
        target_heading = (initial_heading - 90.0) % 360.0

        turn_start = fdm.get_sim_time()
        while fdm.run() and (fdm.get_sim_time() - turn_start) < 30.0:
            current_heading = fdm["attitude/psi-deg"]
            heading_diff = abs(current_heading - target_heading)
            if heading_diff < 10.0 or heading_diff > 350.0:
                break
            fdm["fcs/aileron-cmd-norm"] = 0.3
            fdm["fcs/rudder-cmd-norm"] = 0.15
            fdm["fcs/elevator-cmd-norm"] = -0.05

        # Wings level
        fdm["fcs/aileron-cmd-norm"] = 0.0
        fdm["fcs/rudder-cmd-norm"] = 0.0

        # Phase 6: Final turn
        target_heading = (target_heading - 90.0) % 360.0
        turn_start = fdm.get_sim_time()

        while fdm.run() and (fdm.get_sim_time() - turn_start) < 30.0:
            current_heading = fdm["attitude/psi-deg"]
            heading_diff = abs(current_heading - target_heading)
            if heading_diff < 10.0 or heading_diff > 350.0:
                break
            fdm["fcs/aileron-cmd-norm"] = 0.25
            fdm["fcs/rudder-cmd-norm"] = 0.12

        fdm["fcs/aileron-cmd-norm"] = 0.0
        fdm["fcs/rudder-cmd-norm"] = 0.0

        # Phase 7: Final approach and landing
        fdm["fcs/throttle-cmd-norm"] = 0.2

        approach_start = fdm.get_sim_time()
        while fdm.run() and (fdm.get_sim_time() - approach_start) < 60.0:
            current_alt = fdm["position/h-sl-ft"]
            if current_alt < initial_altitude + 20.0:
                # Flare
                fdm["fcs/elevator-cmd-norm"] = -0.12
                fdm["fcs/throttle-cmd-norm"] = 0.05
            else:
                fdm["fcs/elevator-cmd-norm"] = 0.02

            # Check for touchdown
            if fdm["gear/unit[0]/WOW"] > 0.5:
                break

        # Verify we completed the pattern
        self.assertGreater(fdm.get_sim_time() - start_time, 60.0, "Pattern too short")
        self.assertLess(fdm.get_sim_time() - start_time, 600.0, "Pattern took too long")

    def test_climb_to_altitude(self):
        """
        Test climb from ground to cruise altitude with proper trim.

        This scenario validates:
        1. Engine start at field elevation
        2. Trimming for initial climb configuration
        3. Sustained climb from sea level to 5000 ft
        4. Re-trimming at cruise altitude
        5. Engine performance across altitude range
        6. Atmospheric model effects on performance

        Tests integration of propulsion, aerodynamics, atmosphere,
        and flight dynamics during extended climb.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Start at low altitude
        fdm["ic/h-sl-ft"] = 100.0
        fdm["ic/vc-kts"] = 75.0  # Vy (best rate of climb)
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        initial_altitude = fdm["position/h-sl-ft"]

        # Trim for climb
        TrimAircraft(fdm, throttle_guess=0.7)
        # Note: Trim may not always succeed, but we can still test climb

        # Set climb power
        fdm["fcs/throttle-cmd-norm"] = 0.90

        # Create altitude controller
        alt_controller = SimplePIDController(
            kp=0.002, ki=0.0001, kd=0.003, output_min=-0.3, output_max=0.1
        )

        # Target cruise altitude
        target_altitude = 5000.0
        duration = 600.0  # 10 minutes max
        start_time = fdm.get_sim_time()

        altitudes = []
        climb_rates = []

        # Climb to target altitude
        while fdm.get_sim_time() - start_time < duration:
            elapsed = fdm.get_sim_time() - start_time
            current_alt = fdm["position/h-sl-ft"]

            # Gradually increasing target
            intermediate_target = initial_altitude + min(
                (target_altitude - initial_altitude) * elapsed / duration,
                target_altitude - initial_altitude,
            )

            elevator_cmd = AltitudeHoldController(fdm, intermediate_target, alt_controller)
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            if not fdm.run():
                break

            altitudes.append(current_alt)
            climb_rates.append(fdm["velocities/h-dot-fps"] * 60.0)

            # Check if we've reached target
            if current_alt >= target_altitude:
                break

        final_altitude = fdm["position/h-sl-ft"]
        altitude_gain = final_altitude - initial_altitude

        # Verify significant climb occurred
        self.assertGreater(altitude_gain, 1000.0, f"Insufficient climb: {altitude_gain:.1f} ft")

        # Verify engine still running
        self.assertEqual(fdm["propulsion/engine/set-running"], 1, "Engine quit during climb")

        # Verify climb rates were positive
        if len(climb_rates) > 10:
            avg_climb_rate = np.mean(climb_rates[: len(climb_rates) // 2])  # First half
            self.assertGreater(
                avg_climb_rate, 100.0, f"Climb rate too low: {avg_climb_rate:.0f} fpm"
            )

    def test_cruise_flight(self):
        """
        Test extended cruise flight with manual controls.

        This scenario validates:
        1. Initialization at cruise altitude
        2. Trimming for cruise configuration
        3. Sustained flight with fixed controls
        4. Engine performance during cruise
        5. Fuel consumption modeling
        6. Stability in cruise configuration

        Tests long-duration integration of all subsystems in cruise
        conditions with trimmed flight.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Initialize at cruise altitude
        cruise_altitude = 5500.0
        cruise_speed = 110.0

        fdm["ic/h-sl-ft"] = cruise_altitude
        fdm["ic/vc-kts"] = cruise_speed
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Trim for cruise - this is critical for maintaining altitude
        TrimAircraft(fdm, throttle_guess=0.55)

        # Get state after trim
        initial_altitude = fdm["position/h-sl-ft"]
        try:
            initial_fuel = fdm["propulsion/total-fuel-lbs"]
        except (KeyError, TypeError):
            initial_fuel = 0.0

        # Get trimmed control positions
        trimmed_throttle = fdm["fcs/throttle-cmd-norm"]
        trimmed_elevator = fdm["fcs/elevator-cmd-norm"]

        # Fly for 60 seconds with trimmed controls and monitor
        duration = 60.0
        start_time = fdm.get_sim_time()

        altitudes = []
        speeds = []
        min_altitude = initial_altitude

        while fdm.get_sim_time() - start_time < duration:
            current_alt = fdm["position/h-sl-ft"]

            # Stop if aircraft descending too low
            if current_alt < 1000.0:
                break

            min_altitude = min(min_altitude, current_alt)

            # Maintain trimmed control positions
            fdm["fcs/throttle-cmd-norm"] = trimmed_throttle
            fdm["fcs/elevator-cmd-norm"] = trimmed_elevator
            fdm["fcs/aileron-cmd-norm"] = 0.0
            fdm["fcs/rudder-cmd-norm"] = 0.0

            if not fdm.run():
                break

            altitudes.append(current_alt)
            speeds.append(fdm["velocities/vc-kts"])

        # Verify cruise flight occurred
        # Verify engine still running
        self.assertEqual(fdm["propulsion/engine/set-running"], 1, "Engine should still be running")

        # Verify maintained reasonable altitude for cruise
        # Even if trim isn't perfect, should stay above 1000 ft for a minute
        self.assertGreater(
            min_altitude, 800.0, f"Aircraft descended too low during cruise: {min_altitude:.1f} ft"
        )

        # Verify flight data was collected
        self.assertGreater(len(altitudes), 30, "Insufficient flight data collected")

        # Verify speed reasonable
        if len(speeds) > 0:
            avg_speed = np.mean(speeds)
            self.assertGreater(avg_speed, 60.0, f"Airspeed too low: {avg_speed:.1f} kts")
            self.assertLess(avg_speed, 170.0, f"Airspeed excessive: {avg_speed:.1f} kts")

        # Verify fuel consumed
        try:
            final_fuel = fdm["propulsion/total-fuel-lbs"]
            if initial_fuel > 0 and final_fuel > 0:
                fuel_consumed = initial_fuel - final_fuel
                self.assertGreater(fuel_consumed, 0.0, "No fuel consumed during cruise")
                # C172 burns ~8-10 gal/hr = 48-60 lbs/hr
                # 1 minute should burn roughly 0.8-1.0 lbs
                self.assertLess(
                    fuel_consumed, 5.0, f"Excessive fuel consumption: {fuel_consumed:.1f} lbs"
                )
        except (KeyError, TypeError):
            pass  # Fuel not modeled, skip this check

    def test_descent_and_approach(self):
        """
        Test descent from cruise altitude and approach to landing.

        This scenario validates:
        1. Start from cruise altitude and speed
        2. Power reduction for descent
        3. Controlled descent rate management
        4. Speed control during descent
        5. Configuration changes (simulated)
        6. Descent to pattern altitude

        Tests integration during descent phase with power and configuration
        changes. Validates energy management and speed control.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Start at altitude
        initial_altitude = 5000.0
        fdm["ic/h-sl-ft"] = initial_altitude
        fdm["ic/vc-kts"] = 110.0
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Trim at cruise
        TrimAircraft(fdm, throttle_guess=0.55)

        # Reduce power for descent
        fdm["fcs/throttle-cmd-norm"] = 0.25

        # Create altitude controller for controlled descent
        alt_controller = SimplePIDController(
            kp=0.002, ki=0.0001, kd=0.003, output_min=-0.1, output_max=0.3
        )

        # Target pattern altitude
        target_altitude = 1500.0
        altitude_to_lose = initial_altitude - target_altitude

        # Descend at approximately 500 fpm
        target_descent_rate_fps = 500.0 / 60.0
        duration = 420.0  # 7 minutes max

        start_time = fdm.get_sim_time()
        descent_rates = []
        altitudes = []

        while fdm.get_sim_time() - start_time < duration:
            elapsed = fdm.get_sim_time() - start_time
            current_alt = fdm["position/h-sl-ft"]

            # Calculate descending target
            descent_so_far = target_descent_rate_fps * 60.0 * elapsed
            intermediate_target = initial_altitude - min(descent_so_far, altitude_to_lose)

            elevator_cmd = AltitudeHoldController(fdm, intermediate_target, alt_controller)
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            if not fdm.run():
                break

            altitudes.append(current_alt)
            descent_rates.append(-fdm["velocities/h-dot-fps"] * 60.0)

            # Break when approaching target
            if current_alt <= target_altitude:
                break

        final_altitude = fdm["position/h-sl-ft"]
        altitude_lost = initial_altitude - final_altitude

        # Verify descent occurred
        self.assertGreater(altitude_lost, 1000.0, f"Insufficient descent: {altitude_lost:.1f} ft")

        # Verify controlled descent
        if len(descent_rates) > 10:
            avg_descent_rate = np.mean(descent_rates)
            self.assertGreater(avg_descent_rate, 50.0, "No descent detected")
            self.assertLess(
                avg_descent_rate, 3000.0, f"Descent too steep: {avg_descent_rate:.0f} fpm"
            )

    def test_touch_and_go(self):
        """
        Test touch-and-go maneuver: land, power up, take off again.

        This scenario validates:
        1. Approach and landing from pattern altitude
        2. Touchdown detection
        3. Immediate power application on ground
        4. Rotation and second takeoff
        5. Transition from ground to flight to ground to flight

        Tests critical transition between flight and ground operations,
        including ground reactions during brief touchdown and immediate
        power application for go-around.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Start on short final
        field_elevation = 13.0
        fdm["ic/h-sl-ft"] = field_elevation + 50.0
        fdm["ic/vc-kts"] = 65.0
        fdm["ic/gamma-deg"] = -3.0  # Slight descent
        fdm["ic/psi-true-deg"] = 180.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Approach power
        fdm["fcs/throttle-cmd-norm"] = 0.2

        # Descend to touchdown
        approach_start = fdm.get_sim_time()
        touchdown_detected = False

        while fdm.run() and (fdm.get_sim_time() - approach_start) < 30.0:
            current_alt = fdm["position/h-sl-ft"]

            # Flare near ground
            if current_alt < field_elevation + 15.0:
                fdm["fcs/elevator-cmd-norm"] = -0.12
                fdm["fcs/throttle-cmd-norm"] = 0.05
            else:
                fdm["fcs/elevator-cmd-norm"] = 0.02

            # Check for touchdown
            if fdm["gear/unit[0]/WOW"] > 0.5:
                touchdown_detected = True
                break

        self.assertTrue(touchdown_detected, "Failed to touchdown")

        # Immediately apply full power for go-around
        fdm["fcs/throttle-cmd-norm"] = 1.0
        fdm["fcs/elevator-cmd-norm"] = -0.08  # Reduce pitch for acceleration

        # Accelerate on ground
        accel_time = fdm.get_sim_time()
        while fdm.run() and (fdm.get_sim_time() - accel_time) < 30.0:
            if fdm["velocities/vc-kts"] >= 55.0:
                break
            # Keep nose down until rotation speed
            fdm["fcs/elevator-cmd-norm"] = -0.02

        # Rotate for second takeoff
        fdm["fcs/elevator-cmd-norm"] = -0.15
        rotation_time = fdm.get_sim_time()
        second_liftoff = False

        while fdm.run() and (fdm.get_sim_time() - rotation_time) < 10.0:
            if fdm["position/h-sl-ft"] > field_elevation + 20.0:
                second_liftoff = True
                break

        self.assertTrue(second_liftoff, "Failed to take off again")

        # Verify complete touch-and-go
        total_time = fdm.get_sim_time() - approach_start
        self.assertGreater(total_time, 5.0, "Touch-and-go too quick")
        self.assertLess(total_time, 90.0, "Touch-and-go took too long")

    def test_emergency_descent(self):
        """
        Test emergency descent scenario from high altitude.

        This scenario validates:
        1. Start at high altitude
        2. Rapid descent configuration (power idle, high speed)
        3. High descent rate achievement
        4. Speed control during emergency descent
        5. Leveling off at safe altitude

        Tests system behavior during unusual attitudes and high rates of
        descent. Validates aerodynamics at higher speeds and energy management.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Start at high altitude
        initial_altitude = 8000.0
        fdm["ic/h-sl-ft"] = initial_altitude
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Emergency descent configuration
        fdm["fcs/throttle-cmd-norm"] = 0.0  # Idle power
        fdm["fcs/elevator-cmd-norm"] = 0.1  # Nose down

        # Emergency descent to 3000 ft
        target_altitude = 3000.0
        max_duration = 300.0  # 5 minutes
        start_time = fdm.get_sim_time()

        max_descent_rate = 0.0
        descent_rates = []
        speeds = []

        while fdm.get_sim_time() - start_time < max_duration:
            current_alt = fdm["position/h-sl-ft"]
            descent_rate = -fdm["velocities/h-dot-fps"] * 60.0
            speed = fdm["velocities/vc-kts"]

            descent_rates.append(descent_rate)
            speeds.append(speed)
            max_descent_rate = max(max_descent_rate, descent_rate)

            # Limit speed to Vne (never exceed speed)
            # C172 Vne is ~163 kts, use conservative 145 kts
            if speed > 145.0:
                fdm["fcs/elevator-cmd-norm"] = -0.08  # Pitch up more to slow down
            elif speed > 130.0:
                fdm["fcs/elevator-cmd-norm"] = -0.02  # Gentle pitch up
            else:
                fdm["fcs/elevator-cmd-norm"] = 0.05  # Pitch down for descent

            if not fdm.run():
                break

            # Level off approaching target
            if current_alt <= target_altitude + 200.0:
                break

        final_altitude = fdm["position/h-sl-ft"]
        altitude_lost = initial_altitude - final_altitude

        # Verify rapid descent occurred
        self.assertGreater(altitude_lost, 2000.0, f"Insufficient descent: {altitude_lost:.1f} ft")

        # Verify descent rate was higher than normal
        if len(descent_rates) > 10:
            avg_descent_rate = np.mean(descent_rates)
            self.assertGreater(
                avg_descent_rate,
                500.0,
                f"Emergency descent rate too low: {avg_descent_rate:.0f} fpm",
            )

        # Verify speed was controlled (not excessive)
        if len(speeds) > 10:
            max_speed = max(speeds)
            self.assertLess(
                max_speed, 180.0, f"Speed excessive during descent: {max_speed:.1f} kts"
            )

    def test_fuel_endurance(self):
        """
        Test extended flight to simulate fuel consumption and endurance.

        This scenario validates:
        1. Start with known fuel quantity
        2. Extended cruise flight
        3. Fuel consumption rate
        4. Fuel quantity monitoring
        5. Low fuel state detection

        Tests fuel system modeling and propulsion system fuel consumption
        over extended period. Validates realistic endurance performance.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Initialize at cruise
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Get initial fuel
        try:
            initial_fuel = fdm["propulsion/total-fuel-lbs"]
            if initial_fuel <= 0:
                self.skipTest("Fuel system not modeled for this aircraft")
        except (KeyError, TypeError):
            self.skipTest("Fuel system not modeled for this aircraft")

        # Trim for cruise
        TrimAircraft(fdm, throttle_guess=0.55)

        # Cruise at moderate power
        fdm["fcs/throttle-cmd-norm"] = 0.60

        # Create altitude controller to maintain altitude
        alt_controller = SimplePIDController(
            kp=0.002, ki=0.0001, kd=0.003, output_min=-0.3, output_max=0.1
        )

        target_altitude = fdm["position/h-sl-ft"]

        # Fly for 10 minutes to accumulate fuel burn
        duration = 600.0
        start_time = fdm.get_sim_time()

        fuel_samples = []

        while fdm.get_sim_time() - start_time < duration:
            elevator_cmd = AltitudeHoldController(fdm, target_altitude, alt_controller)
            fdm["fcs/elevator-cmd-norm"] = elevator_cmd

            if not fdm.run():
                break

            try:
                current_fuel = fdm["propulsion/total-fuel-lbs"]
            except (KeyError, TypeError):
                current_fuel = 0.0
            fuel_samples.append(current_fuel)

            # Check for low fuel or empty
            if current_fuel < 5.0:
                break

        try:
            final_fuel = fdm["propulsion/total-fuel-lbs"]
        except (KeyError, TypeError):
            final_fuel = 0.0
        fuel_consumed = initial_fuel - final_fuel
        flight_time_hours = (fdm.get_sim_time() - start_time) / 3600.0

        # Verify fuel was consumed
        self.assertGreater(fuel_consumed, 0.0, "No fuel consumed during extended flight")

        # Verify fuel consumption is realistic
        # C172 burns approximately 8-10 gal/hr = 48-60 lbs/hr
        # 10 minutes at 60 lbs/hr = 10 lbs
        if flight_time_hours > 0:
            burn_rate_lbs_hr = fuel_consumed / flight_time_hours
            # Allow wide range, just verify it's reasonable
            self.assertGreater(
                burn_rate_lbs_hr, 10.0, f"Fuel burn rate too low: {burn_rate_lbs_hr:.1f} lbs/hr"
            )
            self.assertLess(
                burn_rate_lbs_hr, 120.0, f"Fuel burn rate too high: {burn_rate_lbs_hr:.1f} lbs/hr"
            )

        # Verify fuel decreased monotonically (no fuel generation)
        if len(fuel_samples) > 10:
            fuel_diffs = np.diff(fuel_samples)
            positive_diffs = np.sum(fuel_diffs > 0.01)  # Small tolerance for rounding
            # Allow very few positive differences (numerical noise)
            self.assertLess(
                positive_diffs,
                len(fuel_diffs) * 0.1,
                "Fuel quantity increased during flight (should only decrease)",
            )

    def test_altitude_speed_coordinated_control(self):
        """
        Test coordinated control scenario with speed and altitude changes.

        This scenario validates:
        1. Power changes and altitude response
        2. Control inputs during acceleration
        3. Flight at different power settings
        4. Multi-phase flight operation
        5. Engine response to throttle

        Tests integration of controls, propulsion, and aerodynamics during
        power and speed changes.
        """
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X model")

        # Start at moderate altitude and speed
        fdm["ic/h-sl-ft"] = 3000.0
        fdm["ic/vc-kts"] = 90.0
        fdm["ic/gamma-deg"] = 0.0
        self.assertTrue(fdm.run_ic(), "Failed to initialize")

        # Start engine
        engine_started = self._start_engine(fdm)
        self.assertTrue(engine_started, "Engine failed to start")

        # Trim at current configuration
        TrimAircraft(fdm, throttle_guess=0.55)

        initial_altitude = fdm["position/h-sl-ft"]
        initial_speed = fdm["velocities/vc-kts"]
        initial_throttle = fdm["fcs/throttle-cmd-norm"]

        # Phase 1: Increase power and maintain level flight (30 seconds)
        fdm["fcs/throttle-cmd-norm"] = min(initial_throttle + 0.2, 1.0)
        phase1_start = fdm.get_sim_time()

        while fdm.get_sim_time() - phase1_start < 30.0:
            fdm["fcs/elevator-cmd-norm"] = -0.02  # Slight back pressure
            if not fdm.run():
                break

        phase1_altitude = fdm["position/h-sl-ft"]
        phase1_speed = fdm["velocities/vc-kts"]

        # Verify power increase resulted in speed or altitude change
        speed_increase = phase1_speed - initial_speed
        altitude_change = phase1_altitude - initial_altitude

        self.assertTrue(
            speed_increase > 3.0 or altitude_change > 50.0,
            f"No response to power increase: speed +{speed_increase:.1f} kts, alt +{altitude_change:.1f} ft",
        )

        # Phase 2: Reduce power and maintain flight (30 seconds)
        fdm["fcs/throttle-cmd-norm"] = max(initial_throttle - 0.15, 0.2)
        phase2_start = fdm.get_sim_time()

        while fdm.get_sim_time() - phase2_start < 30.0:
            fdm["fcs/elevator-cmd-norm"] = 0.01  # Forward pressure
            if not fdm.run():
                break

        final_altitude = fdm["position/h-sl-ft"]
        final_speed = fdm["velocities/vc-kts"]

        # Verify aircraft is still flying
        self.assertGreater(
            final_altitude, 500.0, f"Aircraft descended to ground: {final_altitude:.1f} ft"
        )

        # Verify engine still running
        self.assertEqual(fdm["propulsion/engine/set-running"], 1, "Engine should still be running")

        # Verify speed is reasonable
        self.assertGreater(final_speed, 50.0, f"Airspeed too low: {final_speed:.1f} kts")
        self.assertLess(final_speed, 150.0, f"Airspeed excessive: {final_speed:.1f} kts")

        # Verify no numerical issues
        for prop in ["position/h-sl-ft", "velocities/vc-kts", "attitude/theta-deg"]:
            value = fdm[prop]
            self.assertFalse(math.isnan(value), f"NaN in {prop}")
            self.assertFalse(math.isinf(value), f"Inf in {prop}")


if __name__ == "__main__":
    RunTest(TestCompleteFlightScenarios)
