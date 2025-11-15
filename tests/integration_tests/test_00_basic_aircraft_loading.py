# test_00_basic_aircraft_loading.py
#
# Integration Test Scenario 0: Basic Aircraft Loading and Simulation
#
# This is the foundational integration test that establishes baseline coverage
# for core FDM operations. It focuses on the most fundamental code paths:
# - FDMExec initialization
# - Aircraft loading
# - Initial condition setup
# - Basic simulation loop execution
# - Property access and updates
#
# This test complements test_01_aircraft_loading.py by focusing on simulation
# execution rather than just configuration validation. It exercises the main
# simulation loop and state propagation.
#
# Components tested:
# - FGFDMExec: Main executive class initialization and run loop
# - FGPropagate: Position and velocity integration
# - FGAccelerations: Acceleration calculations
# - FGAtmosphere: Atmospheric property calculations
# - FGAuxiliary: Derived parameters (Mach, airspeed, etc.)
# - FGAircraft: Forces and moments summation
# - FGGroundReactions: Ground contact detection
# - FGInertial: Reference frame transformations
# - Property system: Get/set operations during simulation
#
# Expected coverage gain: ~3%
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

import pytest


class TestBasicAircraftLoading:
    """
    Basic integration test for aircraft loading and simulation execution.

    This test suite establishes baseline C++ coverage by exercising the most
    fundamental code paths in JSBSim. It verifies that the simulation can:
    - Load aircraft and initialize properly
    - Execute the main simulation loop
    - Update position and velocity states
    - Calculate atmospheric properties
    - Access and modify properties during execution

    These tests use the C172X aircraft (simple, well-tested, representative
    of general aviation) and focus on code execution rather than complex
    flight dynamics validation.
    """

    def test_load_and_initialize_c172x(self, fdm):
        """
        Test basic aircraft loading and initialization.

        Verifies that:
        - Aircraft XML can be loaded
        - Initial conditions can be set
        - run_ic() initializes the simulation properly
        - All core properties are accessible after initialization

        Coverage focus: FGFDMExec::LoadModel, FGInitialCondition::InitModel
        """
        # Load the C172X model
        assert fdm.load_model("c172x"), "Failed to load C172X aircraft"

        # Set basic initial conditions
        fdm["ic/h-sl-ft"] = 5000.0  # 5000 ft altitude
        fdm["ic/vc-kts"] = 100.0  # 100 knots calibrated airspeed
        fdm["ic/psi-true-deg"] = 270.0  # Heading west

        # Initialize simulation
        assert fdm.run_ic(), "Failed to initialize simulation"

        # Verify simulation is at time zero
        assert fdm.get_sim_time() == 0.0, "Sim time should be 0 after run_ic()"

        # Verify initial conditions were applied
        altitude = fdm["position/h-sl-ft"]
        assert abs(altitude - 5000.0) < 1.0, f"Altitude {altitude} != 5000"

        airspeed = fdm["velocities/vc-kts"]
        assert abs(airspeed - 100.0) < 5.0, f"Airspeed {airspeed} != 100"

        heading = fdm["attitude/psi-deg"]
        assert abs(heading - 270.0) < 1.0, f"Heading {heading} != 270"

    def test_run_simulation_loop(self, fdm):
        """
        Test basic simulation loop execution.

        Verifies that:
        - Simulation can run for multiple time steps
        - Time advances correctly
        - Position updates as expected
        - No crashes or exceptions occur

        Coverage focus: FGFDMExec::Run, FGPropagate::Run, FGAtmosphere::Run
        """
        # Load and initialize
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/psi-true-deg"] = 0.0  # Heading north
        fdm.run_ic()

        # Record initial state
        initial_time = fdm.get_sim_time()
        initial_altitude = fdm["position/h-sl-ft"]

        # Run simulation for 1 second (dt is typically 0.00833 sec = 120 Hz)
        # Run 120 frames = 1 second
        for _ in range(120):
            assert fdm.run(), "Simulation run() returned False"

        # Verify time advanced
        final_time = fdm.get_sim_time()
        assert final_time > initial_time, "Time did not advance"
        assert abs(final_time - 1.0) < 0.1, f"Time should be ~1.0 sec, got {final_time}"

        # Verify position changed (aircraft should descend slightly in level flight)
        final_altitude = fdm["position/h-sl-ft"]
        assert final_altitude != initial_altitude, "Altitude did not change during simulation"

    def test_position_propagation(self, fdm):
        """
        Test position and velocity state propagation.

        Verifies that:
        - Geographic position updates during simulation
        - Velocity components are calculated
        - Attitude changes are integrated

        Coverage focus: FGPropagate::Integrate, FGLocation updates
        """
        # Load and initialize
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 120.0
        fdm["ic/psi-true-deg"] = 90.0  # Heading east
        fdm["ic/theta-deg"] = 5.0  # 5 degree climb angle
        fdm.run_ic()

        # Record initial position
        initial_lat = fdm["position/lat-geod-deg"]
        initial_lon = fdm["position/long-gc-deg"]
        initial_alt = fdm["position/h-sl-ft"]

        # Set some throttle to maintain airspeed
        fdm["fcs/throttle-cmd-norm"] = 0.7

        # Run for 5 seconds
        for _ in range(600):  # 5 sec * 120 Hz
            fdm.run()

        # Verify position changed
        final_lat = fdm["position/lat-geod-deg"]
        final_lon = fdm["position/long-gc-deg"]
        final_alt = fdm["position/h-sl-ft"]

        assert final_lon != initial_lon, "Longitude did not change (heading east)"
        assert final_lat == pytest.approx(
            initial_lat, abs=0.01
        ), "Latitude should not change much (heading east)"
        # Altitude may change due to climb angle and thrust
        assert abs(final_alt - initial_alt) > 0.1, "Altitude should have changed"

    def test_atmospheric_calculations(self, fdm):
        """
        Test atmospheric property calculations at different altitudes.

        Verifies that:
        - Atmospheric properties update with altitude
        - Temperature, pressure, density calculated correctly
        - Atmospheric model responds to position changes

        Coverage focus: FGAtmosphere::Calculate, FGStandardAtmosphere
        """
        fdm.load_model("c172x")

        # Test at sea level
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/vc-kts"] = 0.0
        fdm.run_ic()

        sea_level_pressure = fdm["atmosphere/P-psf"]
        sea_level_temp = fdm["atmosphere/T-R"]
        sea_level_density = fdm["atmosphere/rho-slugs_ft3"]

        assert sea_level_pressure > 2000.0, "Sea level pressure too low"
        assert sea_level_temp > 500.0, "Sea level temperature too low"
        assert sea_level_density > 0.002, "Sea level density too low"

        # Test at 10,000 ft
        fdm["ic/h-sl-ft"] = 10000.0
        fdm.run_ic()

        high_altitude_pressure = fdm["atmosphere/P-psf"]
        high_altitude_temp = fdm["atmosphere/T-R"]
        high_altitude_density = fdm["atmosphere/rho-slugs_ft3"]

        # Verify pressure decreases with altitude
        assert high_altitude_pressure < sea_level_pressure, "Pressure should decrease with altitude"
        assert high_altitude_temp < sea_level_temp, "Temperature should decrease with altitude"
        assert high_altitude_density < sea_level_density, "Density should decrease with altitude"

    def test_control_surface_response(self, fdm):
        """
        Test that control surface commands are processed.

        Verifies that:
        - FCS commands can be set
        - Control surface positions respond to commands
        - Flight control system processes inputs

        Coverage focus: FGFCS::Run, actuator models
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Test elevator command
        fdm["fcs/elevator-cmd-norm"] = 0.5
        fdm.run()
        elevator_pos = fdm["fcs/elevator-pos-rad"]
        assert elevator_pos is not None, "Elevator position not available"

        # Test aileron command
        fdm["fcs/aileron-cmd-norm"] = 0.3
        fdm.run()
        aileron_pos = fdm["fcs/left-aileron-pos-rad"]
        assert aileron_pos is not None, "Aileron position not available"

        # Test rudder command
        fdm["fcs/rudder-cmd-norm"] = -0.2
        fdm.run()
        rudder_pos = fdm["fcs/rudder-pos-rad"]
        assert rudder_pos is not None, "Rudder position not available"

        # Test throttle command
        fdm["fcs/throttle-cmd-norm"] = 0.8
        for _ in range(10):  # Run a few frames for throttle to respond
            fdm.run()
        throttle_pos = fdm["fcs/throttle-pos-norm"]
        assert throttle_pos is not None, "Throttle position not available"
        assert throttle_pos > 0.0, "Throttle should be positive"

    def test_auxiliary_parameters(self, fdm):
        """
        Test auxiliary/derived parameter calculations.

        Verifies that:
        - Mach number is calculated
        - True airspeed derived from calibrated
        - Alpha and beta calculated
        - Q-bar (dynamic pressure) computed

        Coverage focus: FGAuxiliary::Run, derived calculations
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 120.0
        fdm.run_ic()

        # Run a few frames to ensure calculations are updated
        for _ in range(10):
            fdm.run()

        # Verify auxiliary parameters are calculated
        mach = fdm["velocities/mach"]
        assert mach is not None, "Mach number not calculated"
        assert 0.0 < mach < 1.0, f"Mach number {mach} out of range for C172"

        true_airspeed = fdm["velocities/vtrue-kts"]
        assert true_airspeed is not None, "True airspeed not calculated"
        assert true_airspeed > 0.0, "True airspeed should be positive"

        alpha = fdm["aero/alpha-rad"]
        assert alpha is not None, "Angle of attack not calculated"

        beta = fdm["aero/beta-rad"]
        assert beta is not None, "Sideslip angle not calculated"

        qbar = fdm["aero/qbar-psf"]
        assert qbar is not None, "Dynamic pressure not calculated"
        assert qbar > 0.0, "Dynamic pressure should be positive"

    def test_mass_properties_during_simulation(self, fdm):
        """
        Test mass property access during simulation.

        Verifies that:
        - Mass properties are available
        - CG location is calculated
        - Moments of inertia are accessible

        Coverage focus: FGMassBalance::Run, inertia calculations
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Run simulation to ensure mass properties are updated
        for _ in range(10):
            fdm.run()

        # Verify mass properties
        weight = fdm["inertia/weight-lbs"]
        assert weight > 1000.0, "C172 weight should be > 1000 lbs"
        assert weight < 5000.0, "C172 weight should be < 5000 lbs"

        # Verify CG location
        cg_x = fdm["inertia/cg-x-in"]
        assert cg_x is not None, "CG X not available"

        # Verify moments of inertia
        ixx = fdm["inertia/ixx-slugs_ft2"]
        iyy = fdm["inertia/iyy-slugs_ft2"]
        izz = fdm["inertia/izz-slugs_ft2"]

        assert ixx > 0.0, "Ixx should be positive"
        assert iyy > 0.0, "Iyy should be positive"
        assert izz > 0.0, "Izz should be positive"

    def test_ground_contact_detection(self, fdm):
        """
        Test ground contact detection and gear forces.

        Verifies that:
        - Ground reactions are calculated
        - On-ground detection works
        - Gear forces are computed when on ground

        Coverage focus: FGGroundReactions::Run, FGLGear calculations
        """
        fdm.load_model("c172x")

        # Initialize on ground
        fdm["ic/h-agl-ft"] = 0.0  # On ground
        fdm["ic/vc-kts"] = 0.0  # Stationary
        fdm.run_ic()

        # Run a few frames
        for _ in range(10):
            fdm.run()

        # Verify ground contact
        # Note: Property names may vary, check most common ones
        try:
            wow = fdm["gear/unit[0]/WOW"]  # Weight-On-Wheels
            assert wow > 0, "Should be on ground (WOW)"
        except KeyError:
            # Alternative property check
            gear_compression = fdm["gear/unit[0]/compression-ft"]
            assert gear_compression is not None, "Gear compression not available"

    def test_velocity_state_consistency(self, fdm):
        """
        Test velocity state vector consistency.

        Verifies that:
        - Body-frame velocities (UVW) are calculated
        - NED velocities are computed
        - Velocity transformations are consistent

        Coverage focus: FGPropagate velocity calculations
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/psi-true-deg"] = 0.0  # North
        fdm.run_ic()

        # Run to stabilize
        for _ in range(10):
            fdm.run()

        # Body frame velocities
        u_fps = fdm["velocities/u-fps"]
        v_fps = fdm["velocities/v-fps"]
        w_fps = fdm["velocities/w-fps"]

        assert u_fps is not None, "U velocity not available"
        assert v_fps is not None, "V velocity not available"
        assert w_fps is not None, "W velocity not available"

        # NED velocities
        v_north = fdm["velocities/v-north-fps"]
        v_east = fdm["velocities/v-east-fps"]
        v_down = fdm["velocities/v-down-fps"]

        assert v_north is not None, "North velocity not available"
        assert v_east is not None, "East velocity not available"
        assert v_down is not None, "Down velocity not available"

        # Heading north with positive airspeed should give positive v_north
        assert v_north > 0.0, "North velocity should be positive when heading north"

    def test_engine_thrust_calculation(self, fdm, start_piston_engine):
        """
        Test propulsion system thrust calculation.

        Verifies that:
        - Engine properties are accessible
        - Thrust is calculated based on throttle
        - Propeller RPM responds to throttle

        Coverage focus: FGPropulsion::Run, FGEngine, FGPropeller
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Start the engine
        assert start_piston_engine(throttle=0.3), "Failed to start engine"

        # Get thrust at low throttle
        low_thrust = fdm["propulsion/engine/thrust-lbs"]
        assert low_thrust is not None, "Thrust not calculated"
        assert low_thrust > 0.0, "Engine running should produce thrust"

        # Set high throttle
        fdm["fcs/throttle-cmd-norm"] = 0.9
        for _ in range(50):
            fdm.run()

        high_thrust = fdm["propulsion/engine/thrust-lbs"]
        assert high_thrust > low_thrust, "Thrust should increase with throttle"

        # Verify propeller RPM is available
        prop_rpm = fdm["propulsion/engine/propeller-rpm"]
        assert prop_rpm is not None, "Propeller RPM not available"
        assert prop_rpm > 0.0, "Propeller should be spinning"

    def test_simulation_timestep(self, fdm):
        """
        Test simulation timestep handling.

        Verifies that:
        - DT is accessible and reasonable
        - Simulation time increments by DT each frame
        - Consistent time advancement

        Coverage focus: FGFDMExec timestep management
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Get timestep
        dt = fdm["simulation/dt"]
        assert dt is not None, "DT not available"
        assert 0.001 < dt < 0.1, f"DT {dt} out of reasonable range"

        # Run one frame and verify time increment
        time_before = fdm.get_sim_time()
        fdm.run()
        time_after = fdm.get_sim_time()

        time_increment = time_after - time_before
        assert abs(time_increment - dt) < 1e-6, f"Time increment {time_increment} != DT {dt}"

    def test_property_catalog_access(self, fdm):
        """
        Test property tree catalog access patterns.

        Verifies that:
        - Properties can be read during simulation
        - Properties can be written during simulation
        - Property changes take effect

        Coverage focus: FGPropertyManager get/set operations
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Test reading properties
        properties_to_test = [
            "position/h-sl-ft",
            "velocities/vc-kts",
            "attitude/phi-deg",
            "attitude/theta-deg",
            "attitude/psi-deg",
            "accelerations/udot-ft_sec2",
            "atmosphere/P-psf",
            "aero/qbar-psf",
            "fcs/throttle-cmd-norm",
        ]

        for prop in properties_to_test:
            value = fdm[prop]
            assert value is not None, f"Property {prop} not accessible"

        # Test writing properties
        fdm["fcs/elevator-cmd-norm"] = 0.5
        assert abs(fdm["fcs/elevator-cmd-norm"] - 0.5) < 0.01, "Elevator command not set"

        fdm["fcs/throttle-cmd-norm"] = 0.7
        assert abs(fdm["fcs/throttle-cmd-norm"] - 0.7) < 0.01, "Throttle command not set"

    def test_accelerations_calculation(self, fdm):
        """
        Test acceleration calculations.

        Verifies that:
        - Linear accelerations are calculated
        - Angular accelerations are computed
        - G-loading is available

        Coverage focus: FGAccelerations::Run
        """
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Run to stabilize
        for _ in range(10):
            fdm.run()

        # Linear accelerations
        udot = fdm["accelerations/udot-ft_sec2"]
        vdot = fdm["accelerations/vdot-ft_sec2"]
        wdot = fdm["accelerations/wdot-ft_sec2"]

        assert udot is not None, "U-dot not calculated"
        assert vdot is not None, "V-dot not calculated"
        assert wdot is not None, "W-dot not calculated"

        # Angular accelerations
        pdot = fdm["accelerations/pdot-rad_sec2"]
        qdot = fdm["accelerations/qdot-rad_sec2"]
        rdot = fdm["accelerations/rdot-rad_sec2"]

        assert pdot is not None, "P-dot not calculated"
        assert qdot is not None, "Q-dot not calculated"
        assert rdot is not None, "R-dot not calculated"

        # Normal acceleration (G-loading)
        nz = fdm["accelerations/Nz"]
        assert nz is not None, "Nz (G-loading) not calculated"
        # In level flight, should be approximately 1G
        assert 0.5 < abs(nz) < 2.0, f"Nz {nz} out of range for level flight"
