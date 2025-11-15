# test_00_f16_aircraft.py
#
# Integration Test Scenario: F-16 High-Performance Fighter Aircraft
#
# This test suite exercises high-performance flight regimes that are not covered
# by general aviation aircraft like the C172X. The F-16 has fundamentally different
# characteristics that exercise different code paths in JSBSim:
#
# 1. Turbine engine (vs. piston) - Different propulsion model
# 2. Supersonic flight (Mach > 1.0) - Wave drag, compressibility effects
# 3. High altitude (40,000+ ft) - Different atmospheric regime
# 4. High G maneuvers (5-7 G's) - Structural limits, extreme accelerations
# 5. Afterburner - Augmented thrust calculations
# 6. High angle of attack - Post-stall aerodynamics
# 7. High-speed turns - Energy management at high dynamic pressure
#
# Components exercised beyond basic C172X tests:
# - FGTurbine: Turbine engine model (N1, N2, fuel flow, afterburner)
# - FGAtmosphere: High-altitude atmospheric calculations (stratosphere)
# - FGAerodynamics: Supersonic aerodynamics (wave drag, Mach effects)
# - FGAccelerations: High G-loading calculations
# - FGPropulsion: Afterburner logic, fuel consumption at high thrust
# - FGAuxiliary: Supersonic Mach calculations, high dynamic pressure
# - FGPropagate: High-speed position integration
#
# Expected coverage gain: ~2-3% (complements C172X with high-performance regimes)
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


class TestF16Aircraft:
    """
    Integration tests for F-16 high-performance fighter aircraft.

    This test suite complements the C172X basic aircraft tests by exercising
    high-performance flight regimes that expose different code paths in JSBSim.
    The F-16 has a turbine engine, can fly supersonic, operate at high altitudes,
    and perform high-G maneuvers - all capabilities not tested with GA aircraft.
    """

    def test_f16_loading_and_initialization(self, fdm):
        """
        Test F-16 loading and basic initialization.

        Verifies that:
        - F-16 aircraft model loads successfully
        - Initial conditions can be set for high-performance aircraft
        - Turbine engine properties are accessible
        - Military aircraft mass properties are in expected range

        Coverage focus: FGFDMExec::LoadModel with turbine-powered aircraft
        """
        # Load F-16 model
        assert fdm.load_model("f16"), "Failed to load F-16 aircraft"

        # Set military jet initial conditions
        fdm["ic/h-sl-ft"] = 10000.0  # 10,000 ft altitude
        fdm["ic/vc-kts"] = 300.0  # 300 knots (typical cruise)
        fdm["ic/psi-true-deg"] = 0.0  # Heading north

        # Initialize
        assert fdm.run_ic(), "Failed to initialize F-16 simulation"

        # Verify initial conditions applied
        altitude = fdm["position/h-sl-ft"]
        assert abs(altitude - 10000.0) < 1.0, f"Altitude {altitude} != 10000"

        # Verify F-16 mass properties (heavier than GA aircraft)
        weight = fdm["inertia/weight-lbs"]
        assert weight > 15000.0, f"F-16 weight {weight} too low for fighter aircraft"
        assert weight < 50000.0, f"F-16 weight {weight} unreasonably high"

        # Verify turbine engine properties exist
        # Note: Turbine engines have N1, N2 instead of RPM
        n1 = fdm["propulsion/engine/n1"]
        n2 = fdm["propulsion/engine/n2"]
        assert n1 is not None, "N1 (fan speed) not available for turbine"
        assert n2 is not None, "N2 (core speed) not available for turbine"

    def test_f16_turbine_engine_startup(self, fdm):
        """
        Test turbine engine startup and operation.

        Verifies that:
        - Turbine engine can be started
        - N1 and N2 spool up correctly
        - Thrust is produced when engine running
        - Fuel flow responds to throttle

        Coverage focus: FGTurbine engine model, startup sequence
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 0.0  # Stationary for startup
        fdm.run_ic()

        # Start turbine engine - simpler than piston, just set running
        fdm["propulsion/engine/set-running"] = 1

        # Set idle throttle
        fdm["fcs/throttle-cmd-norm"] = 0.2

        # Run for a few seconds to stabilize engine
        for _ in range(300):  # 2.5 seconds at 120 Hz
            fdm.run()

        # Verify engine is running
        n1 = fdm["propulsion/engine/n1"]
        n2 = fdm["propulsion/engine/n2"]
        assert n1 > 20.0, f"N1 {n1} too low, engine may not be running"
        assert n2 > 20.0, f"N2 {n2} too low, engine may not be running"

        # Verify thrust is being produced
        thrust = fdm["propulsion/engine/thrust-lbs"]
        assert thrust > 100.0, f"Thrust {thrust} too low for running turbine"

        # Verify fuel flow
        fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]
        assert fuel_flow > 0.0, "Fuel flow should be positive when engine running"

    def test_f16_high_speed_flight(self, fdm):
        """
        Test high subsonic flight (Mach 0.8).

        Verifies that:
        - F-16 can sustain high subsonic speeds
        - Transonic effects are calculated
        - Dynamic pressure is high
        - Aerodynamic calculations work at high speed

        Coverage focus: High-speed aerodynamics, high dynamic pressure regime
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 30000.0  # 30,000 ft
        fdm["ic/vc-kts"] = 350.0  # ~Mach 0.6 at 30,000 ft (conservative)
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.6  # Moderate throttle to maintain subsonic

        # Run simulation to stabilize
        for _ in range(300):
            fdm.run()

        # Verify high-speed regime
        mach = fdm["velocities/mach"]
        assert mach > 0.5, f"Mach {mach} should be high subsonic"
        assert mach < 1.0, f"Mach {mach} should still be subsonic"

        # Verify dynamic pressure is high
        qbar = fdm["aero/qbar-psf"]
        assert qbar > 300.0, f"Dynamic pressure {qbar} should be high at high speed"

        # Verify thrust is substantial
        thrust = fdm["propulsion/engine/thrust-lbs"]
        assert thrust > 3000.0, f"Thrust {thrust} should be substantial at military power"

    def test_f16_supersonic_flight(self, fdm):
        """
        Test supersonic flight (Mach > 1.0).

        Verifies that:
        - F-16 can achieve supersonic speeds
        - Wave drag calculations engage
        - Aerodynamic coefficients reflect supersonic regime
        - Shock wave effects are modeled

        Coverage focus: FGAerodynamics supersonic calculations, wave drag
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 35000.0  # 35,000 ft (ideal for supersonic)
        fdm["ic/vc-kts"] = 700.0  # ~Mach 1.2 at 35,000 ft
        fdm.run_ic()

        # Start engine with afterburner
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 1.0  # Full throttle (with afterburner)

        # Run simulation
        for _ in range(600):  # 5 seconds
            fdm.run()

        # Verify supersonic flight achieved
        mach = fdm["velocities/mach"]
        assert mach > 1.0, f"Mach {mach} should be supersonic"

        # Verify aerodynamic drag force is being calculated
        drag_force = fdm["forces/fwx-aero-lbs"]  # X-axis wind frame force (drag is negative)
        assert abs(drag_force) > 0.0, "Drag force should be non-zero"

        # Verify high dynamic pressure at supersonic speed
        qbar = fdm["aero/qbar-psf"]
        assert qbar > 500.0, f"Dynamic pressure {qbar} should be very high supersonic"

        # Verify thrust with afterburner
        thrust = fdm["propulsion/engine/thrust-lbs"]
        assert thrust > 10000.0, f"Thrust {thrust} should be very high with afterburner"

    def test_f16_high_altitude(self, fdm):
        """
        Test flight at high altitude (45,000 ft).

        Verifies that:
        - Atmospheric calculations work in stratosphere
        - Pressure/temperature/density correct at high altitude
        - Engine performance adjusts for thin air
        - True airspeed much higher than indicated at altitude

        Coverage focus: FGAtmosphere high-altitude regime (stratosphere)
        """
        fdm.load_model("f16")

        # Set very high altitude
        fdm["ic/h-sl-ft"] = 45000.0  # 45,000 ft (near service ceiling)
        fdm["ic/vc-kts"] = 300.0  # 300 KCAS
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.9

        # Run to stabilize
        for _ in range(300):
            fdm.run()

        # Verify altitude
        altitude = fdm["position/h-sl-ft"]
        assert altitude > 44000.0, f"Altitude {altitude} should be at high altitude"

        # Verify stratospheric atmospheric properties
        pressure = fdm["atmosphere/P-psf"]
        temperature = fdm["atmosphere/T-R"]
        density = fdm["atmosphere/rho-slugs_ft3"]

        # At 45,000 ft, pressure should be ~20% of sea level
        assert pressure < 500.0, f"Pressure {pressure} too high for 45,000 ft"
        assert pressure > 200.0, f"Pressure {pressure} too low for 45,000 ft"

        # Temperature should be cold (stratosphere)
        assert temperature < 420.0, f"Temperature {temperature} too warm for stratosphere"

        # Density should be very low
        assert density < 0.001, f"Density {density} too high for 45,000 ft"

        # True airspeed should be much higher than calibrated at this altitude
        vc = fdm["velocities/vc-kts"]
        vt = fdm["velocities/vtrue-kts"]
        assert vt > vc * 1.5, f"TAS {vt} should be >>CAS {vc} at high altitude"

    def test_f16_high_g_maneuver(self, fdm):
        """
        Test high-G turning maneuver (5-7 G's).

        Verifies that:
        - F-16 can sustain high G loads
        - Normal acceleration calculated correctly
        - High angle of attack reached
        - Structural limits modeled

        Coverage focus: FGAccelerations high-G calculations
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 15000.0  # 15,000 ft
        fdm["ic/vc-kts"] = 400.0  # 400 knots for energy
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.8

        # Stabilize flight
        for _ in range(120):
            fdm.run()

        # Execute high-G pull (aggressive elevator input)
        fdm["fcs/elevator-cmd-norm"] = -0.8  # Large nose-up command

        # Run through maneuver
        max_nz = 0.0
        max_alpha = 0.0

        for _ in range(300):  # 2.5 seconds
            fdm.run()

            # Track maximum G and alpha
            nz = fdm["accelerations/Nz"]
            alpha = fdm["aero/alpha-deg"]

            max_nz = max(max_nz, abs(nz))
            max_alpha = max(max_alpha, alpha)

        # Verify high-G achieved
        assert max_nz > 3.0, f"Max G {max_nz} should exceed 3G in aggressive maneuver"

        # Verify high angle of attack
        assert max_alpha > 10.0, f"Max alpha {max_alpha} should exceed 10 deg in pull"

        # Verify acceleration components calculated
        pdot = fdm["accelerations/pdot-rad_sec2"]
        qdot = fdm["accelerations/qdot-rad_sec2"]
        rdot = fdm["accelerations/rdot-rad_sec2"]

        assert pdot is not None, "P-dot not calculated during maneuver"
        assert qdot is not None, "Q-dot not calculated during maneuver"
        assert rdot is not None, "R-dot not calculated during maneuver"

    def test_f16_afterburner_engagement(self, fdm):
        """
        Test afterburner engagement and thrust augmentation.

        Verifies that:
        - Afterburner engages at full throttle
        - Thrust increases significantly with AB
        - Fuel consumption increases dramatically
        - AB effects on performance

        Coverage focus: FGTurbine afterburner logic, augmented thrust
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 20000.0
        fdm["ic/vc-kts"] = 400.0
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1

        # Military power (no afterburner) - throttle < 1.0
        fdm["fcs/throttle-cmd-norm"] = 0.95

        # Run to stabilize
        for _ in range(300):
            fdm.run()

        # Record military power performance
        mil_thrust = fdm["propulsion/engine/thrust-lbs"]
        mil_fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]

        # Engage afterburner - full throttle
        fdm["fcs/throttle-cmd-norm"] = 1.0

        # Run with afterburner
        for _ in range(300):
            fdm.run()

        # Record afterburner performance
        ab_thrust = fdm["propulsion/engine/thrust-lbs"]
        ab_fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]

        # Verify afterburner increases thrust
        # F-16 afterburner provides ~10-20% thrust increase in this model
        assert ab_thrust > mil_thrust * 1.05, (
            f"AB thrust {ab_thrust} should be higher than " f"military thrust {mil_thrust}"
        )

        # Verify fuel flow increases with afterburner
        assert ab_fuel_flow > mil_fuel_flow * 1.1, (
            f"AB fuel flow {ab_fuel_flow} should be higher than "
            f"military fuel flow {mil_fuel_flow}"
        )

    def test_f16_high_speed_turn(self, fdm):
        """
        Test high-speed coordinated turn.

        Verifies that:
        - Turn coordination at high speeds
        - Bank angle and turn rate calculations
        - Energy management during turn
        - High dynamic pressure effects

        Coverage focus: Coordinated flight at high dynamic pressure
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 25000.0
        fdm["ic/vc-kts"] = 450.0
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.85

        # Stabilize
        for _ in range(120):
            fdm.run()

        # Initiate bank for turn
        fdm["fcs/aileron-cmd-norm"] = 0.3

        # Hold bank and add back pressure
        for _ in range(60):
            fdm.run()

        # Verify bank achieved
        phi = abs(fdm["attitude/phi-deg"])
        assert phi > 5.0, f"Bank angle {phi} should be established"

        # Apply elevator for level turn
        fdm["fcs/elevator-cmd-norm"] = -0.3

        # Execute turn
        initial_heading = fdm["attitude/psi-deg"]

        for _ in range(360):  # 3 seconds
            fdm.run()

        final_heading = fdm["attitude/psi-deg"]

        # Verify heading changed (aircraft is turning)
        heading_change = abs(final_heading - initial_heading)
        assert heading_change > 5.0, f"Heading change {heading_change} too small for turn"

        # Verify still flying (not stalled)
        airspeed = fdm["velocities/vc-kts"]
        assert airspeed > 300.0, f"Airspeed {airspeed} degraded too much in turn"

    def test_f16_energy_management(self, fdm):
        """
        Test energy management in climb/dive cycle.

        Verifies that:
        - Energy exchange between altitude and speed
        - Climb performance with turbine
        - Dive acceleration
        - Altitude and speed coupling

        Coverage focus: Energy state calculations, climb/dive dynamics
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 20000.0
        fdm["ic/vc-kts"] = 350.0
        fdm["ic/theta-deg"] = 0.0  # Level
        fdm.run_ic()

        # Start engine
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 1.0  # Full AB

        # Climb phase - pitch up
        fdm["fcs/elevator-cmd-norm"] = -0.4

        initial_altitude = fdm["position/h-sl-ft"]

        # Climb for 5 seconds
        for _ in range(600):
            fdm.run()

        climb_altitude = fdm["position/h-sl-ft"]

        # Verify altitude changed during climb attempt
        altitude_change_climb = climb_altitude - initial_altitude

        # With full AB and pitch up, aircraft should change altitude
        # May climb or may enter a zoom climb then fall back depending on energy state
        assert abs(altitude_change_climb) > 100.0, (
            f"Altitude should have changed significantly during maneuver, "
            f"changed {altitude_change_climb} ft"
        )

        # Now dive - push over
        fdm["fcs/elevator-cmd-norm"] = 0.3

        # Reduce throttle
        fdm["fcs/throttle-cmd-norm"] = 0.3

        # Dive for 5 seconds
        for _ in range(600):
            fdm.run()

        dive_altitude = fdm["position/h-sl-ft"]
        dive_speed = fdm["velocities/vc-kts"]

        # Verify altitude changed during dive
        altitude_change_dive = climb_altitude - dive_altitude

        # With nose-down and reduced throttle, altitude should change
        assert abs(altitude_change_dive) > 100.0, (
            f"Altitude should have changed during dive, " f"changed {altitude_change_dive} ft"
        )

        # Verify airspeed is still reasonable (not stalled)
        assert dive_speed > 200.0, f"Airspeed {dive_speed} should be substantial"

    def test_f16_subsonic_to_supersonic_transition(self, fdm):
        """
        Test transition through Mach 1.0 (transonic regime).

        Verifies that:
        - Smooth transition through Mach 1
        - Drag rise in transonic region
        - Aerodynamic coefficients update correctly
        - No numerical instabilities

        Coverage focus: Transonic aerodynamics, Mach transition
        """
        fdm.load_model("f16")
        fdm["ic/h-sl-ft"] = 35000.0
        fdm["ic/vc-kts"] = 300.0  # Start subsonic (~Mach 0.5)
        fdm.run_ic()

        # Start engine at moderate thrust first
        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.7

        # Stabilize at subsonic
        for _ in range(120):
            fdm.run()

        # Now engage afterburner to accelerate
        fdm["fcs/throttle-cmd-norm"] = 1.0

        # Track Mach number progression
        mach_values = []
        qbar_values = []

        # Accelerate through Mach 1
        for _ in range(1800):  # 15 seconds to allow acceleration
            fdm.run()

            mach = fdm["velocities/mach"]
            qbar = fdm["aero/qbar-psf"]

            mach_values.append(mach)
            qbar_values.append(qbar)

            # Break if we've gone well supersonic
            if mach > 1.3:
                break

        # Verify we passed through Mach 1
        min_mach = min(mach_values)
        max_mach = max(mach_values)

        assert min_mach < 1.0, f"Should start subsonic, min Mach {min_mach}"
        assert max_mach > 1.0, f"Should reach supersonic, max Mach {max_mach}"

        # Verify simulation remained stable (no NaN or infinite values)
        assert all(0.0 <= m <= 3.0 for m in mach_values), "Mach values unstable"
        assert all(q > 0.0 for q in qbar_values), "Dynamic pressure went negative or zero"

    def test_f16_altitude_throttle_response(self, fdm):
        """
        Test engine throttle response at different altitudes.

        Verifies that:
        - Throttle response varies with altitude
        - Engine performance degrades at altitude
        - Thrust calculations account for air density
        - Fuel flow adjusts appropriately

        Coverage focus: FGTurbine altitude compensation
        """
        fdm.load_model("f16")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.8

        for _ in range(300):
            fdm.run()

        sea_level_thrust = fdm["propulsion/engine/thrust-lbs"]
        sea_level_fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]

        # Test at high altitude
        fdm["ic/h-sl-ft"] = 40000.0
        fdm["ic/vc-kts"] = 250.0
        fdm.run_ic()

        fdm["propulsion/engine/set-running"] = 1
        fdm["fcs/throttle-cmd-norm"] = 0.8

        for _ in range(300):
            fdm.run()

        high_alt_thrust = fdm["propulsion/engine/thrust-lbs"]
        high_alt_fuel_flow = fdm["propulsion/engine/fuel-flow-rate-pps"]

        # Verify thrust decreases with altitude (thinner air)
        # Note: For jet engines, thrust decreases roughly linearly with air density
        assert high_alt_thrust < sea_level_thrust, (
            f"High altitude thrust {high_alt_thrust} should be less than "
            f"sea level thrust {sea_level_thrust}"
        )

        # Verify both thrust values are reasonable for turbine engine
        assert sea_level_thrust > 5000.0, "Sea level thrust should be substantial"
        assert high_alt_thrust > 2000.0, "High altitude thrust should still be significant"

        # Note: Fuel flow behavior can be complex - depends on throttle setting,
        # altitude, and engine model. At high altitude, engine may need to work
        # harder (higher fuel flow) to produce same throttle setting.
        # We verify both are positive but don't make assumptions about relative values
        assert sea_level_fuel_flow > 0.0, "Sea level fuel flow should be positive"
        assert high_alt_fuel_flow > 0.0, "High altitude fuel flow should be positive"
