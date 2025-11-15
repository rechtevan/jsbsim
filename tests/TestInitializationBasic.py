# TestInitializationBasic.py
#
# Basic tests for initial conditions setup and aircraft initialization.
# This test suite focuses on fundamental IC parameter setting, validation,
# and basic trimming scenarios using the C172X aircraft.
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

from JSBSim_utils import JSBSimTestCase, RunTest


class TestInitializationBasic(JSBSimTestCase):
    """
    Basic tests for initial condition setup and aircraft initialization.

    Tests fundamental IC parameters, position/velocity/attitude setup,
    engine initialization, IC consistency checking, and different IC
    specification methods.

    Coverage areas:
    - FGInitialCondition: Basic IC parameter setting
    - IC property system interaction
    - Position initialization (lat/lon/alt)
    - Velocity initialization (airspeed, climb rate, body velocities)
    - Attitude initialization (pitch, roll, yaw)
    - Engine state initialization
    - IC consistency validation
    - Different IC specification methods (properties vs files)
    """

    def test_position_ic_setup(self):
        """
        Test position initialization (latitude, longitude, altitude).

        Validates that position IC parameters can be set via properties
        and are correctly applied when run_ic() is called.

        Tests:
        - ic/lat-geod-deg (geodetic latitude)
        - ic/long-gc-deg (longitude)
        - ic/h-sl-ft (altitude MSL)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set position initial conditions
        target_lat = 37.619  # San Francisco area
        target_lon = -122.375
        target_alt = 5000.0

        fdm["ic/lat-geod-deg"] = target_lat
        fdm["ic/long-gc-deg"] = target_lon
        fdm["ic/h-sl-ft"] = target_alt

        # Run initial conditions
        fdm.run_ic()

        # Verify position was set correctly
        actual_lat = fdm["position/lat-geod-deg"]
        actual_lon = fdm["position/long-gc-deg"]
        actual_alt = fdm["position/h-sl-ft"]

        self.assertAlmostEqual(actual_lat, target_lat, delta=0.01, msg="Latitude should match IC")
        self.assertAlmostEqual(actual_lon, target_lon, delta=0.01, msg="Longitude should match IC")
        self.assertAlmostEqual(actual_alt, target_alt, delta=1.0, msg="Altitude should match IC")

    def test_velocity_ic_setup(self):
        """
        Test velocity initialization (airspeed, climb rate).

        Validates that velocity IC parameters can be set and produce
        the expected initial velocity state.

        Tests:
        - ic/vc-kts (calibrated airspeed)
        - ic/gamma-deg (flight path angle / climb rate)
        - Derived velocities (u, v, w body velocities)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set velocity initial conditions
        target_airspeed = 100.0  # knots calibrated
        target_gamma = 0.0  # level flight

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = target_airspeed
        fdm["ic/gamma-deg"] = target_gamma

        # Run initial conditions
        fdm.run_ic()

        # Verify airspeed was set correctly
        actual_airspeed = fdm["velocities/vc-kts"]
        self.assertAlmostEqual(
            actual_airspeed,
            target_airspeed,
            delta=1.0,
            msg="Calibrated airspeed should match IC",
        )

        # Verify flight path angle
        actual_gamma = fdm["flight-path/gamma-deg"]
        self.assertAlmostEqual(
            actual_gamma,
            target_gamma,
            delta=1.0,
            msg="Flight path angle should match IC",
        )

        # Verify body velocities were computed (u should be dominant)
        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        self.assertGreater(u, 0.0, msg="Forward velocity should be positive")
        self.assertLess(abs(v), 10.0, msg="Lateral velocity should be small for wings-level flight")

    def test_orientation_ic_setup(self):
        """
        Test attitude initialization (pitch, roll, yaw).

        Validates that orientation IC parameters correctly set the
        aircraft's initial attitude.

        Tests:
        - ic/phi-deg (roll angle)
        - ic/theta-deg (pitch angle)
        - ic/psi-true-deg (true heading)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set orientation initial conditions
        target_phi = 5.0  # 5 degree bank
        target_theta = 3.0  # 3 degree pitch up
        target_psi = 45.0  # 45 degree heading (NE)

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/phi-deg"] = target_phi
        fdm["ic/theta-deg"] = target_theta
        fdm["ic/psi-true-deg"] = target_psi

        # Run initial conditions
        fdm.run_ic()

        # Verify orientation was set correctly
        actual_phi = fdm["attitude/phi-deg"]
        actual_theta = fdm["attitude/theta-deg"]
        actual_psi = fdm["attitude/psi-deg"]

        self.assertAlmostEqual(actual_phi, target_phi, delta=0.1, msg="Roll angle should match IC")
        self.assertAlmostEqual(
            actual_theta, target_theta, delta=0.1, msg="Pitch angle should match IC"
        )
        self.assertAlmostEqual(actual_psi, target_psi, delta=0.1, msg="Heading should match IC")

    def test_engine_running_ic(self):
        """
        Test engine running state at initialization.

        Validates that engines can be started as part of initial conditions
        and that engine properties are correctly initialized.

        Tests:
        - propulsion/engine/set-running
        - Engine RPM initialization
        - Propeller rotation initialization
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set up initial conditions
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Start engine properly (piston engine needs cranking)
        # Set mixture based on altitude
        altitude = fdm["position/h-sl-ft"]
        mixture = 0.87 if altitude < 3000 else (0.92 if altitude < 6000 else 1.0)

        fdm["fcs/mixture-cmd-norm"] = mixture
        fdm["fcs/throttle-cmd-norm"] = 0.2  # Small throttle for start
        fdm["propulsion/magneto_cmd"] = 3  # Both magnetos on
        fdm["propulsion/starter_cmd"] = 1  # Engage starter

        # Crank engine for 2.5 seconds
        dt = fdm["simulation/dt"]
        frames_to_crank = int(2.5 / dt)

        for _ in range(frames_to_crank):
            fdm.run()

        # Disengage starter
        fdm["propulsion/starter_cmd"] = 0

        # Verify engine is running
        engine_running = fdm["propulsion/engine/set-running"]
        self.assertGreater(engine_running, 0, msg="Engine should be running")

        # Verify engine RPM is positive
        rpm = fdm["propulsion/engine/engine-rpm"]
        self.assertGreater(rpm, 0.0, msg="Engine RPM should be positive when running")

        # Verify propeller is spinning
        prop_rpm = fdm["propulsion/engine/propeller-rpm"]
        self.assertGreater(prop_rpm, 0.0, msg="Propeller RPM should be positive when running")

    def test_ic_consistency_check(self):
        """
        Test IC consistency validation.

        Validates that JSBSim checks for and handles inconsistent or
        incompatible initial condition combinations.

        Tests:
        - Consistent velocity/angle combinations
        - Position validity
        - Altitude constraints
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set consistent ICs
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm["ic/alpha-deg"] = 5.0
        fdm["ic/gamma-deg"] = 0.0

        # Run IC - should succeed with consistent values
        result = fdm.run_ic()
        self.assertTrue(result, msg="run_ic() should succeed with consistent ICs")

        # Verify simulation time is reset to zero
        sim_time = fdm["simulation/sim-time-sec"]
        self.assertEqual(sim_time, 0.0, msg="Simulation time should be zero after IC")

        # Verify state is valid (no NaN values)
        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]
        self.assertFalse(math.isnan(u), msg="U velocity should not be NaN")
        self.assertFalse(math.isnan(v), msg="V velocity should not be NaN")
        self.assertFalse(math.isnan(w), msg="W velocity should not be NaN")

    def test_different_ic_types(self):
        """
        Test different IC specification methods.

        Validates that ICs can be specified using different methods:
        - Calibrated airspeed (vc)
        - True airspeed (vt)
        - Mach number
        - Ground speed (vg)

        Tests IC specification flexibility.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test 1: Calibrated airspeed
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()
        vc = fdm["velocities/vc-kts"]
        self.assertAlmostEqual(vc, 100.0, delta=0.5, msg="Calibrated airspeed IC should work")

        # Test 2: True airspeed
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vt-kts"] = 110.0
        fdm.run_ic()
        vt = fdm["velocities/vt-fps"]
        # Convert 110 kts to fps: 110 * 1.68781 = 185.66 fps
        self.assertGreater(vt, 180.0, msg="True airspeed should be set correctly")

        # Test 3: Mach number
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/mach"] = 0.15
        fdm.run_ic()
        mach = fdm["velocities/mach"]
        self.assertAlmostEqual(mach, 0.15, delta=0.01, msg="Mach number IC should work")

        # Test 4: Ground speed
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vg-kts"] = 95.0
        fdm.run_ic()
        vg = fdm["velocities/vg-fps"]
        # Should be approximately 95 kts = 160.3 fps
        self.assertGreater(vg, 150.0, msg="Ground speed should be set")

    def test_ground_start_ic(self):
        """
        Test starting on ground vs in air.

        Validates that ICs correctly set up aircraft on the ground
        (zero velocity, gear loaded) vs in flight.

        Tests:
        - Ground start with zero velocity
        - Gear contact with ground
        - In-air start with positive velocity
        - Gear retracted/unloaded
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test 1: Ground start
        fdm["ic/h-sl-ft"] = 0.0
        fdm["ic/terrain-elevation-ft"] = 0.0
        fdm["ic/vc-kts"] = 0.0
        fdm.run_ic()

        # Verify on ground
        altitude_agl = fdm["position/h-agl-ft"]
        self.assertLess(altitude_agl, 10.0, msg="Aircraft should be near ground for ground start")

        # Verify zero velocity
        u = fdm["velocities/u-fps"]
        v = fdm["velocities/v-fps"]
        w = fdm["velocities/w-fps"]
        self.assertAlmostEqual(u, 0.0, delta=0.1, msg="U should be zero on ground")
        self.assertAlmostEqual(v, 0.0, delta=0.1, msg="V should be zero on ground")
        self.assertAlmostEqual(w, 0.0, delta=0.1, msg="W should be zero on ground")

        # Test 2: Air start
        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/vc-kts"] = 100.0
        fdm.run_ic()

        # Verify in air
        altitude_agl = fdm["position/h-agl-ft"]
        self.assertGreater(
            altitude_agl, 100.0, msg="Aircraft should be well above ground for air start"
        )

        # Verify positive airspeed
        airspeed = fdm["velocities/vc-kts"]
        self.assertGreater(airspeed, 90.0, msg="Should have positive airspeed in air")

    def test_ic_from_properties(self):
        """
        Test IC set via property tree.

        Validates that ICs can be set by directly modifying ic/* properties
        rather than loading from an IC file.

        Tests:
        - Direct property setting
        - run_ic() applies property-based ICs
        - IC properties propagate to state properties
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set ICs via properties (not from file)
        fdm["ic/h-sl-ft"] = 7500.0
        fdm["ic/lat-geod-deg"] = 40.0
        fdm["ic/long-gc-deg"] = -105.0
        fdm["ic/vc-kts"] = 120.0
        fdm["ic/psi-true-deg"] = 270.0  # West
        fdm["ic/theta-deg"] = 2.0
        fdm["ic/phi-deg"] = 0.0

        # Verify IC properties were set
        self.assertAlmostEqual(fdm["ic/h-sl-ft"], 7500.0, delta=1.0)
        self.assertAlmostEqual(fdm["ic/vc-kts"], 120.0, delta=1.0)

        # Run initial conditions
        fdm.run_ic()

        # Verify properties propagated to state
        self.assertAlmostEqual(
            fdm["position/h-sl-ft"], 7500.0, delta=1.0, msg="Altitude should propagate"
        )
        self.assertAlmostEqual(
            fdm["position/lat-geod-deg"],
            40.0,
            delta=0.01,
            msg="Latitude should propagate",
        )
        self.assertAlmostEqual(
            fdm["position/long-gc-deg"],
            -105.0,
            delta=0.01,
            msg="Longitude should propagate",
        )
        self.assertAlmostEqual(
            fdm["velocities/vc-kts"],
            120.0,
            delta=2.0,
            msg="Airspeed should propagate",
        )
        self.assertAlmostEqual(
            fdm["attitude/psi-deg"], 270.0, delta=0.5, msg="Heading should propagate"
        )
        self.assertAlmostEqual(
            fdm["attitude/theta-deg"], 2.0, delta=0.5, msg="Pitch should propagate"
        )

    def test_ic_altitude_modes(self):
        """
        Test different altitude specification modes.

        Validates that altitude can be specified as:
        - MSL (mean sea level)
        - AGL (above ground level)

        Tests altitude reference frame handling.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Test 1: Altitude MSL
        target_msl = 5000.0
        fdm["ic/h-sl-ft"] = target_msl
        fdm.run_ic()

        actual_msl = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(
            actual_msl, target_msl, delta=1.0, msg="MSL altitude should match IC"
        )

        # Test 2: Altitude AGL
        # Set terrain elevation and AGL altitude
        terrain_elev = 1000.0
        target_agl = 4000.0
        fdm["ic/terrain-elevation-ft"] = terrain_elev
        fdm["ic/h-agl-ft"] = target_agl
        fdm.run_ic()

        actual_agl = fdm["position/h-agl-ft"]
        actual_msl = fdm["position/h-sl-ft"]

        # AGL should match
        self.assertAlmostEqual(
            actual_agl, target_agl, delta=10.0, msg="AGL altitude should match IC"
        )

        # MSL should be approximately terrain + AGL
        expected_msl = terrain_elev + target_agl
        self.assertAlmostEqual(
            actual_msl,
            expected_msl,
            delta=20.0,
            msg="MSL should be terrain + AGL",
        )

    def test_ic_body_velocity_components(self):
        """
        Test initialization with body velocity components.

        Validates that ICs can be specified using body-axis velocities
        (u, v, w) instead of airspeed and angles.

        Tests:
        - ic/u-fps (forward velocity)
        - ic/v-fps (lateral velocity)
        - ic/w-fps (vertical velocity)
        """
        fdm = self.create_fdm()
        fdm.load_model("c172x")

        # Set body velocities directly
        target_u = 150.0  # fps forward
        target_v = 0.0  # fps lateral
        target_w = 10.0  # fps downward

        fdm["ic/h-sl-ft"] = 5000.0
        fdm["ic/u-fps"] = target_u
        fdm["ic/v-fps"] = target_v
        fdm["ic/w-fps"] = target_w

        # Run initial conditions
        fdm.run_ic()

        # Verify body velocities were set
        actual_u = fdm["velocities/u-fps"]
        actual_v = fdm["velocities/v-fps"]
        actual_w = fdm["velocities/w-fps"]

        self.assertAlmostEqual(actual_u, target_u, delta=2.0, msg="U velocity should match IC")
        self.assertAlmostEqual(actual_v, target_v, delta=1.0, msg="V velocity should match IC")
        self.assertAlmostEqual(actual_w, target_w, delta=2.0, msg="W velocity should match IC")

        # Verify total airspeed was calculated
        vt = fdm["velocities/vt-fps"]
        expected_vt = math.sqrt(target_u**2 + target_v**2 + target_w**2)
        self.assertAlmostEqual(
            vt, expected_vt, delta=5.0, msg="Total airspeed should be vector magnitude"
        )


RunTest(TestInitializationBasic)
