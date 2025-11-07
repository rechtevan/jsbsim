# test_04_climb_descent.py
#
# Integration Test Scenario 4: Climb and Descent Maneuvers
#
# This test exercises JSBSim's ability to simulate realistic climb and descent
# maneuvers for general aviation aircraft. It validates the physics of vertical
# flight, engine performance under varying power settings, and energy conservation
# principles (potential energy ↔ kinetic energy).
#
# Components tested:
# - FGPropagate: Integration of vertical velocity and altitude changes
# - FGAccelerations: Computation of vertical accelerations
# - FGAtmosphere: Atmospheric properties at varying altitudes
# - FGPropulsion: Engine performance under different power settings and altitudes
# - FGAerodynamics: Aerodynamic forces during climb/descent (lift, drag)
# - FGMassBalance: Weight and energy calculations
# - FGInitialCondition: Setting up climb/descent initial states
# - FGAuxiliary: Derived parameters (climb rate, energy state)
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

import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from JSBSim_utils import JSBSimTestCase, RunTest


class TestClimbDescent(JSBSimTestCase):
    """
    Integration test for aircraft climb and descent maneuvers.

    This test suite verifies that JSBSim accurately simulates vertical flight
    dynamics including climb rate, descent rate, power-altitude relationships,
    and energy conservation. Tests use the Cessna 172P as a representative
    general aviation aircraft with well-documented performance characteristics.

    Test Coverage:
    - Steady-state climb performance
    - Controlled descent operations
    - Power setting effects on altitude
    - Energy conservation (potential ↔ kinetic)
    - Atmospheric effects at varying altitudes
    - Engine performance degradation with altitude
    """

    def test_steady_climb_from_sea_level(self):
        """
        Test steady climb from sea level to 8000 ft.

        This test verifies that the C172P can perform a realistic steady climb
        from sea level to 8000 feet while maintaining:
        - Realistic climb rate (700-900 fpm at sea level, decreasing with altitude)
        - Appropriate airspeed (Vy = 73-76 kts for best rate of climb)
        - Engine parameters within normal operating range
        - Positive altitude gain throughout the maneuver

        The test validates:
        - FGPropagate: Altitude integration during climb
        - FGPropulsion: Engine thrust at climb power settings
        - FGAtmosphere: Atmospheric density changes with altitude
        - FGAerodynamics: Lift generation during positive climb
        """
        fdm = self.create_fdm()

        # Load C172P model
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P aircraft model")

        # Set initial conditions: sea level, best climb speed
        fdm["ic/h-sl-ft"] = 0.0  # Sea level
        fdm["ic/vc-kts"] = 75.0  # Vy (best rate of climb speed)
        fdm["ic/gamma-deg"] = 0.0  # Level flight initially
        fdm["ic/psi-true-deg"] = 0.0  # North heading

        # Initialize the simulation
        self.assertTrue(fdm.run_ic(), "Failed to initialize for climb test")

        # Start engine and set climb power
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 1.0  # Full throttle for climb
        fdm["fcs/mixture-cmd-norm"] = 0.87  # Mixture for altitude

        # Run a few steps to stabilize
        for _ in range(50):
            fdm.run()

        # Record initial altitude
        initial_altitude = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(
            initial_altitude, 0.0, delta=100.0, msg="Initial altitude not at sea level"
        )

        # Set climb attitude with elevator trim
        fdm["fcs/elevator-cmd-norm"] = -0.1  # Slight back pressure

        # Simulate climb for approximately 10 minutes (600 seconds)
        # dt is typically 1/120 sec, so we need ~72000 steps for 600 sec
        # For testing, we'll sample periodically
        sim_time = 0.0
        dt = fdm.get_delta_t()
        target_time = 600.0  # 10 minutes
        steps_between_checks = 1200  # Check every ~10 seconds

        climb_rates = []
        altitudes = []
        airspeeds = []

        step_count = 0
        while sim_time < target_time:
            fdm.run()
            sim_time = fdm.get_sim_time()
            step_count += 1

            # Sample data periodically
            if step_count % steps_between_checks == 0:
                altitude = fdm["position/h-sl-ft"]
                airspeed = fdm["velocities/vc-kts"]
                climb_rate = fdm["velocities/h-dot-fps"] * 60.0  # Convert to fpm

                altitudes.append(altitude)
                airspeeds.append(airspeed)
                climb_rates.append(climb_rate)

                # If we've reached 8000 ft, we're done
                if altitude >= 8000.0:
                    break

        # Final altitude should be significantly higher
        final_altitude = fdm["position/h-sl-ft"]
        altitude_gain = final_altitude - initial_altitude

        self.assertGreater(altitude_gain, 5000.0, "Insufficient altitude gain during climb")
        self.assertLessEqual(final_altitude, 9000.0, "Climbed too high (overshoot)")

        # Verify climb rates are in realistic range
        # C172 at sea level: ~700-900 fpm, decreasing with altitude
        if len(climb_rates) > 0:
            avg_climb_rate = sum(climb_rates) / len(climb_rates)
            self.assertGreater(
                avg_climb_rate, 200.0, "Average climb rate too low (likely stalled or no climb)"
            )
            self.assertLess(
                avg_climb_rate, 1500.0, "Average climb rate unrealistically high for C172"
            )

        # Verify airspeed remained in reasonable range
        if len(airspeeds) > 0:
            avg_airspeed = sum(airspeeds) / len(airspeeds)
            self.assertGreater(avg_airspeed, 60.0, "Airspeed too low (near stall)")
            self.assertLess(avg_airspeed, 100.0, "Airspeed too high for best climb")

        # Verify altitude increased monotonically (no descents during climb)
        for i in range(1, len(altitudes)):
            self.assertGreaterEqual(
                altitudes[i], altitudes[i - 1] - 50.0, f"Unexpected altitude loss at sample {i}"
            )

        # Verify engine is still running
        engine_running = fdm["propulsion/engine/set-running"]
        self.assertEqual(engine_running, 1, "Engine failed during climb")

    def test_controlled_descent(self):
        """
        Test controlled descent from 8000 ft to 3000 ft.

        This test verifies that the C172P can perform a controlled descent
        while maintaining:
        - Controlled descent rate (not exceeding structural limits)
        - Safe airspeed (within normal operating range)
        - Engine at reduced power (idle or partial power)
        - Smooth altitude loss without oscillations

        The test validates:
        - FGPropagate: Altitude integration during descent
        - FGPropulsion: Engine at reduced power settings
        - FGAerodynamics: Drag management during descent
        - FGFCS: Flight control response during descent
        """
        fdm = self.create_fdm()

        # Load C172P model
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P aircraft model")

        # Set initial conditions: 8000 ft, cruise speed
        fdm["ic/h-sl-ft"] = 8000.0
        fdm["ic/vc-kts"] = 90.0  # Normal cruise speed
        fdm["ic/gamma-deg"] = 0.0  # Level flight initially
        fdm["ic/psi-true-deg"] = 0.0  # North heading

        # Initialize the simulation
        self.assertTrue(fdm.run_ic(), "Failed to initialize for descent test")

        # Start engine at reduced power for descent
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.3  # Reduced power for descent
        fdm["fcs/mixture-cmd-norm"] = 0.87

        # Run a few steps to stabilize
        for _ in range(50):
            fdm.run()

        # Record initial altitude
        initial_altitude = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(
            initial_altitude, 8000.0, delta=200.0, msg="Initial altitude not at 8000 ft"
        )

        # Set descent attitude with elevator
        fdm["fcs/elevator-cmd-norm"] = 0.05  # Slight forward pressure

        # Simulate descent for approximately 10 minutes
        sim_time = 0.0
        target_time = 600.0  # 10 minutes
        steps_between_checks = 1200  # Check every ~10 seconds

        descent_rates = []
        altitudes = []
        airspeeds = []

        step_count = 0
        while sim_time < target_time:
            fdm.run()
            sim_time = fdm.get_sim_time()
            step_count += 1

            # Sample data periodically
            if step_count % steps_between_checks == 0:
                altitude = fdm["position/h-sl-ft"]
                airspeed = fdm["velocities/vc-kts"]
                descent_rate = fdm["velocities/h-dot-fps"] * 60.0  # Convert to fpm

                altitudes.append(altitude)
                airspeeds.append(airspeed)
                descent_rates.append(descent_rate)

                # If we've reached 3000 ft, we're done
                if altitude <= 3000.0:
                    break

        # Final altitude should be significantly lower
        final_altitude = fdm["position/h-sl-ft"]
        altitude_loss = initial_altitude - final_altitude

        self.assertGreater(altitude_loss, 3000.0, "Insufficient altitude loss during descent")
        self.assertGreaterEqual(final_altitude, 2000.0, "Descended too low (overshoot)")

        # Verify descent rates are controlled (negative but not excessive)
        # Typical C172 descent: 300-700 fpm
        if len(descent_rates) > 0:
            avg_descent_rate = sum(descent_rates) / len(descent_rates)
            self.assertLess(avg_descent_rate, 100.0, "Should be descending (negative climb rate)")
            self.assertGreater(avg_descent_rate, -1500.0, "Descent rate too high (unsafe)")

        # Verify airspeed remained safe
        if len(airspeeds) > 0:
            for airspeed in airspeeds:
                self.assertGreater(airspeed, 60.0, "Airspeed dangerously low during descent")
                self.assertLess(airspeed, 140.0, "Airspeed too high during descent")

        # Verify altitude decreased monotonically (no climbs during descent)
        for i in range(1, len(altitudes)):
            self.assertLessEqual(
                altitudes[i], altitudes[i - 1] + 50.0, f"Unexpected altitude gain at sample {i}"
            )

    def test_power_changes_altitude_response(self):
        """
        Test altitude response to power changes.

        This test verifies that altitude responds appropriately to power
        setting changes:
        - Increasing power causes climb
        - Decreasing power causes descent
        - Power changes affect climb/descent rate
        - System stabilizes after power change

        The test validates:
        - FGPropulsion: Thrust response to throttle changes
        - FGPropagate: Altitude changes due to thrust variations
        - FGAerodynamics: Trim changes with power
        - System dynamics and stability
        """
        fdm = self.create_fdm()

        # Load C172P model
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P aircraft model")

        # Set initial conditions: 5000 ft, cruise speed
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 85.0  # Cruise speed
        fdm["ic/gamma-deg"] = 0.0  # Level flight
        fdm["ic/psi-true-deg"] = 0.0

        # Initialize the simulation
        self.assertTrue(fdm.run_ic(), "Failed to initialize for power change test")

        # Start engine at cruise power
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.65  # Cruise power
        fdm["fcs/mixture-cmd-norm"] = 0.87

        # Stabilize at cruise
        for _ in range(600):  # ~5 seconds
            fdm.run()

        # Record baseline altitude
        baseline_altitude = fdm["position/h-sl-ft"]

        # Phase 1: Increase power to climb power
        fdm["fcs/throttle-cmd-norm"] = 1.0  # Full throttle
        fdm["fcs/elevator-cmd-norm"] = -0.08  # Slight back pressure

        # Run for ~60 seconds and verify climb
        altitudes_climb = []
        for i in range(7200):  # ~60 seconds
            fdm.run()
            if i % 600 == 0:  # Sample every ~5 seconds
                altitudes_climb.append(fdm["position/h-sl-ft"])

        climb_altitude = fdm["position/h-sl-ft"]
        altitude_gain_climb = climb_altitude - baseline_altitude

        self.assertGreater(altitude_gain_climb, 300.0, "Insufficient climb after power increase")

        # Verify altitude increased during climb phase
        for i in range(1, len(altitudes_climb)):
            self.assertGreaterEqual(
                altitudes_climb[i],
                altitudes_climb[i - 1] - 20.0,
                "Altitude should increase during climb phase",
            )

        # Phase 2: Reduce power to descent power
        fdm["fcs/throttle-cmd-norm"] = 0.2  # Low power
        fdm["fcs/elevator-cmd-norm"] = 0.05  # Slight forward pressure

        # Record start of descent
        descent_start_altitude = fdm["position/h-sl-ft"]

        # Run for ~60 seconds and verify descent
        altitudes_descent = []
        for i in range(7200):  # ~60 seconds
            fdm.run()
            if i % 600 == 0:  # Sample every ~5 seconds
                altitudes_descent.append(fdm["position/h-sl-ft"])

        descent_altitude = fdm["position/h-sl-ft"]
        altitude_loss_descent = descent_start_altitude - descent_altitude

        self.assertGreater(
            altitude_loss_descent, 200.0, "Insufficient descent after power reduction"
        )

        # Verify altitude decreased during descent phase
        for i in range(1, len(altitudes_descent)):
            self.assertLessEqual(
                altitudes_descent[i],
                altitudes_descent[i - 1] + 20.0,
                "Altitude should decrease during descent phase",
            )

        # Phase 3: Return to cruise power and stabilize
        fdm["fcs/throttle-cmd-norm"] = 0.65
        fdm["fcs/elevator-cmd-norm"] = 0.0

        # Run for ~30 seconds
        for _ in range(3600):  # ~30 seconds
            fdm.run()

        final_altitude = fdm["position/h-sl-ft"]
        climb_rate_final = fdm["velocities/h-dot-fps"] * 60.0  # fpm

        # At cruise power, climb rate should be small (near zero)
        self.assertLess(
            abs(climb_rate_final), 300.0, "Climb rate should stabilize near zero at cruise power"
        )

    def test_energy_conservation(self):
        """
        Test energy conservation during climb and descent.

        This test verifies that energy is conserved during altitude changes,
        following the principle that potential energy and kinetic energy
        are interchangeable:
        - Climbing: kinetic energy converts to potential energy (speed decreases)
        - Descending: potential energy converts to kinetic energy (speed increases)
        - Total energy changes only due to engine power and drag

        The test validates:
        - FGPropagate: Accurate velocity and position integration
        - FGMassBalance: Correct weight and inertia calculations
        - FGAerodynamics: Realistic drag forces
        - Physics engine: Energy conservation principles
        """
        fdm = self.create_fdm()

        # Load C172P model
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P aircraft model")

        # Set initial conditions: 3000 ft, moderate speed
        fdm["ic/h-sl-ft"] = 3000.0
        fdm["ic/vc-kts"] = 80.0
        fdm["ic/gamma-deg"] = 0.0
        fdm["ic/psi-true-deg"] = 0.0

        # Initialize the simulation
        self.assertTrue(fdm.run_ic(), "Failed to initialize for energy test")

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.65  # Cruise power
        fdm["fcs/mixture-cmd-norm"] = 0.87

        # Stabilize
        for _ in range(600):
            fdm.run()

        # Get aircraft weight for energy calculations
        weight_lbs = fdm["inertia/weight-lbs"]
        weight_slugs = weight_lbs / 32.174  # Convert to slugs

        # Function to calculate total energy
        def calculate_total_energy():
            """Calculate total mechanical energy (potential + kinetic)."""
            altitude_ft = fdm["position/h-sl-ft"]
            velocity_fps = fdm["velocities/vt-fps"]

            # Potential energy: m * g * h
            potential_energy = weight_lbs * altitude_ft  # ft-lbs

            # Kinetic energy: 0.5 * m * v^2
            kinetic_energy = 0.5 * weight_slugs * velocity_fps * velocity_fps  # ft-lbs

            return potential_energy, kinetic_energy, potential_energy + kinetic_energy

        # Record initial energy state
        pe_initial, ke_initial, te_initial = calculate_total_energy()
        altitude_initial = fdm["position/h-sl-ft"]
        speed_initial = fdm["velocities/vt-fps"]

        # Phase 1: Climb with constant power
        # In a climb, we expect:
        # - Altitude increases (PE increases)
        # - Speed might decrease slightly (KE decreases)
        # - Total energy increases (due to engine work)

        fdm["fcs/throttle-cmd-norm"] = 0.90  # High power for climb
        fdm["fcs/elevator-cmd-norm"] = -0.08

        # Climb for ~60 seconds
        for _ in range(7200):
            fdm.run()

        pe_climb, ke_climb, te_climb = calculate_total_energy()
        altitude_climb = fdm["position/h-sl-ft"]
        speed_climb = fdm["velocities/vt-fps"]

        # Verify altitude increased
        altitude_change_climb = altitude_climb - altitude_initial
        self.assertGreater(altitude_change_climb, 200.0, "Altitude should increase during climb")

        # Verify potential energy increased
        pe_change_climb = pe_climb - pe_initial
        self.assertGreater(pe_change_climb, 0.0, "Potential energy should increase during climb")

        # Phase 2: Descent with reduced power
        # In a descent, we expect:
        # - Altitude decreases (PE decreases)
        # - Speed might increase (KE increases)
        # - Total energy decreases (due to drag and reduced engine power)

        fdm["fcs/throttle-cmd-norm"] = 0.25  # Low power
        fdm["fcs/elevator-cmd-norm"] = 0.05

        # Record state at start of descent
        pe_descent_start, ke_descent_start, te_descent_start = calculate_total_energy()
        altitude_descent_start = fdm["position/h-sl-ft"]
        speed_descent_start = fdm["velocities/vt-fps"]

        # Descend for ~60 seconds
        for _ in range(7200):
            fdm.run()

        pe_descent, ke_descent, te_descent = calculate_total_energy()
        altitude_descent = fdm["position/h-sl-ft"]
        speed_descent = fdm["velocities/vt-fps"]

        # Verify altitude decreased
        altitude_change_descent = altitude_descent_start - altitude_descent
        self.assertGreater(
            altitude_change_descent, 200.0, "Altitude should decrease during descent"
        )

        # Verify potential energy decreased
        pe_change_descent = pe_descent_start - pe_descent
        self.assertGreater(
            pe_change_descent, 0.0, "Potential energy should decrease during descent"
        )

        # Energy conservation check:
        # The change in total energy should be consistent with:
        # 1. Engine work (positive)
        # 2. Drag work (negative)
        # We verify that energy changes are reasonable, not that they're exactly conserved
        # (because engine and drag do work on the system)

        # During climb: total energy should increase (engine adds more than drag removes)
        te_change_climb = te_climb - te_initial
        self.assertGreater(
            te_change_climb, 0.0, "Total energy should increase during powered climb"
        )

        # Verify the relationship between PE and KE changes
        # In climb: PE increase should be significant
        pe_ratio_climb = abs(pe_change_climb) / abs(te_change_climb) if te_change_climb != 0 else 0
        self.assertGreater(
            pe_ratio_climb,
            0.3,
            "Significant portion of energy should go to potential energy in climb",
        )

        # Verify energy calculations are physically reasonable
        self.assertGreater(pe_initial, 0.0, "Initial potential energy should be positive")
        self.assertGreater(ke_initial, 0.0, "Initial kinetic energy should be positive")
        self.assertGreater(te_initial, 0.0, "Initial total energy should be positive")


if __name__ == "__main__":
    RunTest(TestClimbDescent)
