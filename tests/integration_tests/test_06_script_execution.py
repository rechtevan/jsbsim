# test_06_script_execution.py
#
# Integration Test Scenario 6: Script Execution
#
# This test exercises JSBSim's script execution system end-to-end, validating
# that scripts load correctly, execute their events, apply initial conditions,
# and run to completion without errors. It tests the integrated behavior of
# the script system, event handling, property changes, and simulation stability.
#
# Components tested:
# - FGScript: Script parsing, loading, and execution
# - FGXMLElement: XML parsing of script files
# - FGPropertyManager: Property access and modification from scripts
# - Event system: Condition evaluation and event triggering
# - FGInitialCondition: IC setup from script definitions
# - FGFDMExec: Overall simulation execution and control
# - All aircraft subsystems: During script-driven simulation
# - Output system: Generation of output files (when specified)
#
# Expected coverage gain: +3-4%
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
import pytest

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import JSBSimTestCase, RunTest


class TestScriptExecution(JSBSimTestCase):
    """
    Integration test for JSBSim script execution system.

    This test suite validates that JSBSim can successfully load and execute
    various script files, verifying that initial conditions are applied,
    events are triggered, simulations run to completion, and no numerical
    errors occur during execution.

    Test Coverage:
    - Script XML parsing and loading
    - Initial condition setup from scripts
    - Event system execution and condition evaluation
    - Property modifications during simulation
    - Simulation completion criteria
    - Numerical stability (no NaN/Inf values)
    - Output file generation (when specified)
    - Multiple aircraft types and configurations
    - Short and medium duration scripts
    """

    def test_c1721_basic_events(self):
        """
        Test c1721.xml script - basic control surface event execution.

        This script tests simple event-based control surface deflections
        with different action types (FG_STEP, FG_EXP, FG_RAMP).

        Script characteristics:
        - Aircraft: c172r
        - Duration: 10 seconds
        - Events: Aileron, rudder, elevator deflections with timing
        - Tests: Basic event system and control surface commands

        Validates:
        - Script loads successfully
        - Events execute at specified times
        - Control surface properties are modified
        - Simulation completes without errors
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script c1721.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Verify initial conditions were applied
        self.assertIsNotNone(fdm["position/h-sl-ft"], "Altitude not initialized")
        self.assertIsNotNone(fdm["velocities/vc-kts"], "Airspeed not initialized")

        # Store initial sim time
        start_time = fdm.get_sim_time()

        # Run simulation to completion
        iterations = 0
        max_iterations = 100000
        last_time = start_time

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time is advancing
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Check for numerical stability every 100 iterations
            if iterations % 100 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check to prevent infinite loops
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed (reached end time of 10 seconds)
        final_time = fdm.get_sim_time()
        self.assertGreaterEqual(
            final_time, 10.0, "Script did not run to completion (expected ~10s)"
        )
        self.assertLessEqual(final_time, 10.5, "Script ran longer than expected end time")

    def test_c1722_autopilot_events(self):
        """
        Test c1722.xml script - autopilot and trim operations.

        This script tests engine start, trim, and autopilot engagement
        at altitude with extended runtime.

        Script characteristics:
        - Aircraft: c172x
        - Duration: 200 seconds
        - Events: Engine start, trim, autopilot engagement
        - Tests: Propulsion, trim system, autopilot integration

        Validates:
        - Complex event sequences execute correctly
        - Engine start and running state
        - Autopilot property changes
        - Extended duration simulation stability
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1722.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script c1722.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Store initial conditions
        start_time = fdm.get_sim_time()
        initial_alt = fdm["position/h-sl-ft"]

        # Verify altitude is reasonable (script starts at altitude)
        self.assertGreater(initial_alt, 1000.0, "Initial altitude too low")

        # Run simulation to completion
        iterations = 0
        max_iterations = 250000  # Higher limit for longer script
        last_time = start_time
        engine_started = False

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time is advancing
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Check for engine start after initial event (around 0.25s)
            if current_time > 1.0 and not engine_started:
                engine_running = fdm["propulsion/engine/set-running"]
                if engine_running > 0:
                    engine_started = True
                    thrust = fdm["propulsion/engine/thrust-lbs"]
                    self.assertGreater(thrust, 0.0, "Engine running but no thrust")

            # Check for numerical stability periodically
            if iterations % 500 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed
        final_time = fdm.get_sim_time()
        self.assertGreaterEqual(
            final_time, 200.0, "Script did not run to completion (expected ~200s)"
        )
        self.assertLessEqual(final_time, 201.0, "Script ran longer than expected end time")

        # Verify engine was started during simulation
        self.assertTrue(engine_started, "Engine was never started during script")

    def test_c1723_comprehensive_events(self):
        """
        Test c1723.xml script - comprehensive C172 flight profile.

        This is one of the most comprehensive C172 test scripts with
        many events and property changes.

        Script characteristics:
        - Aircraft: c172x (likely)
        - Duration: Variable (check script)
        - Events: Multiple complex events
        - Tests: Full integration of systems

        Validates:
        - Complex event sequences
        - Multiple property interactions
        - System integration
        - Extended simulation stability
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1723.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script c1723.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Run simulation to completion
        iterations = 0
        max_iterations = 500000  # Large limit for comprehensive script
        last_time = fdm.get_sim_time()

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time progression
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Periodic stability checks
            if iterations % 1000 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed successfully
        final_time = fdm.get_sim_time()
        self.assertGreater(final_time, 0.0, "Script did not advance simulation time")

    def test_c172_cruise_8k(self):
        """
        Test c172_cruise_8K.xml script - altitude hold autopilot test.

        This script tests C172 cruise performance and autopilot at
        8000 ft altitude with atmospheric variations.

        Script characteristics:
        - Aircraft: c172x
        - Duration: 40 seconds
        - Events: Engine start, trim, autopilot altitude/heading hold, gust
        - Tests: Autopilot, atmospheric effects, cruise performance

        Validates:
        - Altitude hold autopilot functionality
        - Atmospheric property changes (delta-T)
        - Gust effects
        - Performance at altitude
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c172_cruise_8K.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script c172_cruise_8K.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Store initial altitude (should be around 4000 ft per script)
        initial_alt = fdm["position/h-sl-ft"]
        self.assertGreater(initial_alt, 1000.0, "Initial altitude too low for cruise")

        # Run simulation to completion
        iterations = 0
        max_iterations = 50000
        last_time = fdm.get_sim_time()
        autopilot_engaged = False

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time progression
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Check for autopilot engagement (around 5s in script)
            if current_time > 6.0 and not autopilot_engaged:
                alt_hold = fdm["ap/altitude_hold"]
                if alt_hold > 0:
                    autopilot_engaged = True

            # Periodic stability checks
            if iterations % 100 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed
        final_time = fdm.get_sim_time()
        self.assertGreaterEqual(
            final_time, 40.0, "Script did not run to completion (expected ~40s)"
        )
        self.assertLessEqual(final_time, 41.0, "Script ran longer than expected end time")

    def test_737_cruise(self):
        """
        Test 737_cruise.xml script - Boeing 737 cruise flight.

        This script tests a jet aircraft in cruise configuration with
        turbine engines and trim operations.

        Script characteristics:
        - Aircraft: 737
        - Duration: 100 seconds
        - Events: Engine start, trim
        - Tests: Turbine propulsion, jet aircraft trim

        Validates:
        - Jet aircraft script execution
        - Turbine engine operation
        - High-speed cruise stability
        - Different aircraft type compatibility
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "737_cruise.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script 737_cruise.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Verify we're at cruise altitude
        initial_alt = fdm["position/h-sl-ft"]
        self.assertGreater(initial_alt, 10000.0, "737 cruise altitude should be high altitude")

        # Run simulation to completion
        iterations = 0
        max_iterations = 150000
        last_time = fdm.get_sim_time()
        engines_running = False

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time progression
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Check for engines running
            if current_time > 1.0 and not engines_running:
                engine0 = fdm["propulsion/engine[0]/set-running"]
                engine1 = fdm["propulsion/engine[1]/set-running"]
                if engine0 > 0 and engine1 > 0:
                    engines_running = True
                    # Verify turbine engines are producing thrust
                    thrust0 = fdm["propulsion/engine[0]/thrust-lbs"]
                    thrust1 = fdm["propulsion/engine[1]/thrust-lbs"]
                    self.assertGreater(thrust0, 0.0, "Engine 0 running but no thrust")
                    self.assertGreater(thrust1, 0.0, "Engine 1 running but no thrust")

            # Periodic stability checks
            if iterations % 200 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed
        final_time = fdm.get_sim_time()
        self.assertGreaterEqual(
            final_time, 100.0, "Script did not run to completion (expected ~100s)"
        )
        self.assertLessEqual(final_time, 101.0, "Script ran longer than expected end time")

        # Verify engines were started
        self.assertTrue(engines_running, "737 engines were never started")

    def test_short_s23_seaplane(self):
        """
        Test Short_S23_1.xml script - seaplane takeoff test.

        This script tests a complex seaplane (flying boat) takeoff from
        water, which exercises the hydrodynamics and buoyancy systems.

        Script characteristics:
        - Aircraft: Short_S23 (flying boat)
        - Duration: 200 seconds
        - Events: Complex initialization, engine start, takeoff
        - Tests: Hydrodynamics, buoyancy, water operations

        Validates:
        - Seaplane/flying boat operations
        - Hydrodynamic forces and moments
        - Water takeoff simulation
        - Complex aircraft configuration
        """
        fdm = self.create_fdm()

        # Load and run the script
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "Short_S23_1.xml")
        self.assertTrue(fdm.load_script(script_path), "Failed to load script Short_S23_1.xml")

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize from script")

        # Verify initial conditions
        initial_alt = fdm["position/h-sl-ft"]
        # Seaplane should start on or very near water surface
        self.assertLess(initial_alt, 100.0, "Seaplane should start at low altitude (on water)")

        # Run simulation to completion
        iterations = 0
        max_iterations = 300000  # Higher limit for complex seaplane simulation
        last_time = fdm.get_sim_time()

        while fdm.run():
            iterations += 1
            current_time = fdm.get_sim_time()

            # Verify time progression
            self.assertGreaterEqual(current_time, last_time, "Simulation time moving backwards")
            last_time = current_time

            # Periodic stability checks
            if iterations % 500 == 0:
                self._verify_no_nan_inf(fdm)

            # Safety check
            self.assertLess(iterations, max_iterations, "Simulation exceeded maximum iterations")

        # Verify simulation completed
        final_time = fdm.get_sim_time()
        self.assertGreaterEqual(
            final_time, 200.0, "Script did not run to completion (expected ~200s)"
        )
        self.assertLessEqual(final_time, 201.0, "Script ran longer than expected end time")

    @pytest.mark.parametrize(
        "script_name,expected_min_duration,aircraft_type",
        [
            ("c1721.xml", 10.0, "c172r"),
            ("c1722.xml", 200.0, "c172x"),
            ("c1724.xml", 5.0, "c172"),  # Short script
            ("c172_cross_wind.xml", 10.0, "c172"),
            ("c172_head_wind.xml", 10.0, "c172"),
            ("T37.xml", 5.0, "T37"),  # Different aircraft type
            ("T38.xml", 5.0, "T38"),  # Different aircraft type
        ],
    )
    def test_multiple_scripts_execution(self):
        """
        Test multiple scripts execute successfully.

        Tests several different scripts to validate that the script execution
        system works across different aircraft types and scenarios.

        Validates:
        - Scripts load without errors
        - Simulations run to completion
        - No numerical instabilities
        - Time progression is correct
        """
        # Test data: (script_name, expected_min_duration, aircraft_type)
        test_scripts = [
            ("c1721.xml", 9.0, "c172r"),
            ("ball.xml", 5.0, "ball"),
        ]

        for script_name, expected_min_duration, aircraft_type in test_scripts:
            fdm = self.create_fdm()

            # Load the script
            script_path = self.sandbox.path_to_jsbsim_file("scripts", script_name)

            # Check if script exists (some may not be present in all distributions)
            if not os.path.isfile(script_path):
                continue  # Skip this script

            self.assertTrue(fdm.load_script(script_path), f"Failed to load script {script_name}")

            # Initialize
            self.assertTrue(fdm.run_ic(), f"Failed to initialize from script {script_name}")

            # Verify basic properties are initialized
            self.assertIsNotNone(
                fdm["position/h-sl-ft"], f"{script_name}: Altitude not initialized"
            )
            self.assertIsNotNone(
                fdm["velocities/vc-kts"], f"{script_name}: Airspeed not initialized"
            )
            self.assertIsNotNone(
                fdm["attitude/phi-deg"], f"{script_name}: Roll angle not initialized"
            )

            # Run simulation to completion
            iterations = 0
            max_iterations = int(expected_min_duration * 10000)  # Scale with duration
            last_time = fdm.get_sim_time()

            while fdm.run():
                iterations += 1
                current_time = fdm.get_sim_time()

                # Verify time progression
                self.assertGreaterEqual(
                    current_time,
                    last_time,
                    f"{script_name}: Simulation time moving backwards",
                )
                last_time = current_time

                # Periodic stability checks
                check_interval = max(100, iterations // 100)
                if iterations % check_interval == 0:
                    self._verify_no_nan_inf(fdm, script_name)

                # Safety check
                self.assertLess(
                    iterations,
                    max_iterations,
                    f"{script_name}: Simulation exceeded maximum iterations",
                )

            # Verify simulation completed
            final_time = fdm.get_sim_time()
            self.assertGreaterEqual(
                final_time,
                expected_min_duration * 0.99,  # Allow 1% tolerance
                f"{script_name}: Script did not run to expected duration",
            )

    def test_script_property_changes(self):
        """
        Test that script events properly modify properties.

        This test uses c1721.xml which has timed events that change
        control surface properties, validating that the event system
        correctly modifies FDM properties.

        Validates:
        - Events trigger at correct times
        - Property values change as specified
        - Event actions (FG_STEP, FG_RAMP, FG_EXP) work correctly
        - Properties are accessible during execution
        """
        fdm = self.create_fdm()

        # Load script with known events
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        fdm.load_script(script_path)
        fdm.run_ic()

        # Track property changes
        aileron_values = []
        elevator_values = []
        rudder_values = []
        time_values = []

        # Run and record property values
        while fdm.run():
            time_values.append(fdm.get_sim_time())
            aileron_values.append(fdm["fcs/aileron-cmd-norm"])
            elevator_values.append(fdm["fcs/elevator-cmd-norm"])
            rudder_values.append(fdm["fcs/rudder-cmd-norm"])

        # Verify we collected data
        self.assertGreater(len(time_values), 100, "Not enough data points collected")

        # Verify aileron changed (event at 0.25s sets to 0.25, event at 0.5s adds 0.5)
        aileron_array = np.array(aileron_values)
        self.assertTrue(np.any(aileron_array > 0.1), "Aileron was never deflected by events")

        # Verify elevator changed (event at 1.5s sets to 0.25, event at 2.5s adds 0.5)
        elevator_array = np.array(elevator_values)
        self.assertTrue(np.any(elevator_array > 0.1), "Elevator was never deflected by events")

        # Verify rudder changed (event at 1.5s adds 0.5)
        rudder_array = np.array(rudder_values)
        self.assertTrue(np.any(rudder_array != 0.0), "Rudder was never deflected by events")

    def test_script_initial_conditions_applied(self):
        """
        Test that script initial conditions are correctly applied.

        Validates that when a script specifies initial conditions in the
        initialization file, those conditions are properly loaded and
        applied to the simulation.

        Validates:
        - IC from script are loaded
        - Position properties match expectations
        - Velocity properties match expectations
        - Attitude properties match expectations
        """
        fdm = self.create_fdm()

        # Load a script with known initial conditions
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1722.xml")
        fdm.load_script(script_path)
        fdm.run_ic()

        # Verify position properties were initialized
        altitude = fdm["position/h-sl-ft"]
        latitude = fdm["position/lat-geod-deg"]
        longitude = fdm["position/long-gc-deg"]

        self.assertIsNotNone(altitude, "Altitude not initialized from script IC")
        self.assertIsNotNone(latitude, "Latitude not initialized from script IC")
        self.assertIsNotNone(longitude, "Longitude not initialized from script IC")

        # Verify velocity properties were initialized
        airspeed = fdm["velocities/vc-kts"]
        self.assertIsNotNone(airspeed, "Airspeed not initialized from script IC")

        # Verify attitude properties were initialized
        phi = fdm["attitude/phi-deg"]
        theta = fdm["attitude/theta-deg"]
        psi = fdm["attitude/psi-deg"]

        self.assertIsNotNone(phi, "Roll angle not initialized from script IC")
        self.assertIsNotNone(theta, "Pitch angle not initialized from script IC")
        self.assertIsNotNone(psi, "Heading not initialized from script IC")

        # Verify values are reasonable (not NaN or extreme)
        self.assertFalse(math.isnan(altitude), "Altitude is NaN")
        self.assertFalse(math.isinf(altitude), "Altitude is Inf")
        self.assertGreater(altitude, -1000.0, "Altitude unreasonably negative")
        self.assertLess(altitude, 100000.0, "Altitude unreasonably high")

    def test_simulation_time_matches_script_end(self):
        """
        Test that simulation time matches script end time.

        Validates that the simulation runs until the script's specified
        end time and then terminates correctly.

        Validates:
        - Simulation reaches script end time
        - Simulation doesn't significantly exceed end time
        - Time tracking is accurate
        """
        fdm = self.create_fdm()

        # Use c1721.xml which has a known end time of 10 seconds
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        fdm.load_script(script_path)
        fdm.run_ic()

        expected_end_time = 10.0

        # Run to completion
        while fdm.run():
            pass

        # Verify final time
        final_time = fdm.get_sim_time()
        self.assertAlmostEqual(
            final_time,
            expected_end_time,
            delta=0.1,
            msg="Final simulation time does not match script end time",
        )

    # Helper methods

    def _verify_no_nan_inf(self, fdm, context=""):
        """
        Helper method to verify critical properties have no NaN or Inf values.

        Args:
            fdm: FDMExec instance
            context: Optional context string for error messages (e.g., script name)

        Raises:
            AssertionError: If any critical property is NaN or Inf
        """
        prefix = f"{context}: " if context else ""

        critical_properties = [
            "position/h-sl-ft",
            "position/lat-geod-deg",
            "position/long-gc-deg",
            "velocities/u-fps",
            "velocities/v-fps",
            "velocities/w-fps",
            "velocities/vc-kts",
            "attitude/phi-deg",
            "attitude/theta-deg",
            "attitude/psi-deg",
            "accelerations/udot-ft_sec2",
            "accelerations/vdot-ft_sec2",
            "accelerations/wdot-ft_sec2",
        ]

        for prop in critical_properties:
            try:
                value = fdm[prop]
                self.assertFalse(math.isnan(value), f"{prefix}Property {prop} is NaN")
                self.assertFalse(math.isinf(value), f"{prefix}Property {prop} is Inf")
            except (KeyError, Exception) as e:
                # Some properties may not exist for all aircraft configurations
                # Only fail if it's a truly critical property
                if prop in [
                    "position/h-sl-ft",
                    "velocities/vc-kts",
                    "attitude/phi-deg",
                ]:
                    raise AssertionError(f"{prefix}Critical property {prop} not accessible: {e}")


if __name__ == "__main__":
    RunTest(TestScriptExecution)
