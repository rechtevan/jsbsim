# test_01_aircraft_loading.py
#
# Integration Test Scenario 1: Aircraft Loading
#
# This test exercises the foundation of JSBSim by loading multiple aircraft
# types and verifying that all core properties are correctly initialized.
# It validates the XML parsing, property tree initialization, and basic
# aircraft configuration loading.
#
# Components tested:
# - FGFDMExec: Main executive initialization and aircraft loading
# - FGXMLElement: XML parsing and validation
# - FGPropertyManager: Property tree registration and initialization
# - FGAircraft: Aircraft configuration and assembly
# - FGInitialCondition: Initial condition system
# - FGMassBalance: Mass properties initialization
# - FGPropulsion: Engine configuration loading
# - FGAerodynamics: Aerodynamic reference values
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


class TestAircraftLoading(JSBSimTestCase):
    """
    Integration test for aircraft loading and property initialization.

    This test suite verifies that JSBSim can successfully load various
    aircraft types and that all critical properties are correctly initialized.
    It serves as the foundation test that all other integration tests build upon.

    Test Coverage:
    - Multiple aircraft types (single-engine GA, fighter, simple ball, etc.)
    - Property tree navigation and access
    - Mass properties verification
    - Engine configuration validation
    - Aerodynamic reference values
    - XML parsing correctness
    """

    def test_load_c172p(self):
        """
        Test loading Cessna 172P (single-engine GA aircraft).

        The C172P is a representative general aviation aircraft with:
        - Single piston engine
        - Fixed landing gear
        - Simple flight control system
        - Well-documented flight characteristics

        This test verifies that all core properties are initialized correctly
        for a typical general aviation aircraft.
        """
        fdm = self.create_fdm()

        # Load the C172P model
        self.assertTrue(fdm.load_model("c172p"), "Failed to load C172P aircraft model")

        # Verify critical position properties exist and are initialized
        self.assertIsNotNone(fdm["position/h-sl-ft"], "Altitude property not initialized")
        self.assertIsNotNone(fdm["position/lat-geod-deg"], "Latitude property not initialized")
        self.assertIsNotNone(fdm["position/long-gc-deg"], "Longitude property not initialized")

        # Verify velocity properties exist
        self.assertIsNotNone(
            fdm["velocities/vc-kts"], "Calibrated airspeed property not initialized"
        )
        self.assertIsNotNone(fdm["velocities/u-fps"], "Body-frame u velocity not initialized")
        self.assertIsNotNone(fdm["velocities/v-fps"], "Body-frame v velocity not initialized")
        self.assertIsNotNone(fdm["velocities/w-fps"], "Body-frame w velocity not initialized")

        # Verify attitude properties exist
        self.assertIsNotNone(fdm["attitude/phi-deg"], "Roll angle property not initialized")
        self.assertIsNotNone(fdm["attitude/theta-deg"], "Pitch angle property not initialized")
        self.assertIsNotNone(fdm["attitude/psi-deg"], "Yaw angle property not initialized")

        # Verify flight control system properties exist
        self.assertIsNotNone(
            fdm["fcs/throttle-cmd-norm"], "Throttle command property not initialized"
        )
        self.assertIsNotNone(
            fdm["fcs/elevator-cmd-norm"], "Elevator command property not initialized"
        )
        self.assertIsNotNone(
            fdm["fcs/aileron-cmd-norm"], "Aileron command property not initialized"
        )
        self.assertIsNotNone(fdm["fcs/rudder-cmd-norm"], "Rudder command property not initialized")

        # Verify mass properties are loaded correctly
        wing_area = fdm["metrics/Sw-sqft"]
        self.assertIsNotNone(wing_area, "Wing area not loaded")
        self.assertGreater(wing_area, 0.0, "Wing area must be positive")
        self.assertLess(wing_area, 500.0, "Wing area unreasonably large for C172")

        wing_span = fdm["metrics/bw-ft"]
        self.assertIsNotNone(wing_span, "Wing span not loaded")
        self.assertGreater(wing_span, 0.0, "Wing span must be positive")

        wing_chord = fdm["metrics/cbarw-ft"]
        self.assertIsNotNone(wing_chord, "Wing chord not loaded")
        self.assertGreater(wing_chord, 0.0, "Wing chord must be positive")

        # Verify inertia properties
        mass = fdm["inertia/weight-lbs"]
        self.assertIsNotNone(mass, "Aircraft weight not loaded")
        self.assertGreater(mass, 1000.0, "C172 weight should be > 1000 lbs")
        self.assertLess(mass, 5000.0, "C172 weight should be < 5000 lbs")

        # Verify engine properties are loaded
        self.assertIsNotNone(
            fdm["propulsion/engine/thrust-lbs"], "Engine thrust property not initialized"
        )

    def test_load_f16(self):
        """
        Test loading F-16 Fighting Falcon (fighter jet).

        The F-16 is a military fighter aircraft with:
        - Turbine engine
        - Complex flight control system
        - High performance characteristics
        - Advanced aerodynamics

        This test verifies that JSBSim can load more complex aircraft
        configurations including turbine engines and advanced FCS.
        """
        fdm = self.create_fdm()

        # Load the F-16 model
        self.assertTrue(fdm.load_model("f16"), "Failed to load F-16 aircraft model")

        # Verify critical properties exist
        self.assertIsNotNone(fdm["position/h-sl-ft"], "Altitude property not initialized")
        self.assertIsNotNone(
            fdm["velocities/vc-kts"], "Calibrated airspeed property not initialized"
        )
        self.assertIsNotNone(fdm["attitude/phi-deg"], "Roll angle property not initialized")

        # Verify F-16 specific properties
        # F-16 should have higher wing loading than C172
        wing_area = fdm["metrics/Sw-sqft"]
        self.assertIsNotNone(wing_area, "Wing area not loaded")
        self.assertGreater(wing_area, 100.0, "F-16 wing area should be > 100 sqft")

        mass = fdm["inertia/weight-lbs"]
        self.assertIsNotNone(mass, "Aircraft weight not loaded")
        self.assertGreater(mass, 10000.0, "F-16 weight should be > 10,000 lbs")

        # F-16 has turbine engine - verify thrust property
        self.assertIsNotNone(
            fdm["propulsion/engine/thrust-lbs"], "Turbine thrust property not initialized"
        )

    def test_load_ball(self):
        """
        Test loading the 'ball' aircraft (simple test model).

        The ball is a minimalist test aircraft with:
        - Spherical aerodynamics
        - No propulsion system
        - Simple mass properties
        - Useful for testing basic physics

        This test verifies that even the simplest aircraft configurations
        load correctly and all required properties are initialized.
        """
        fdm = self.create_fdm()

        # Load the ball model
        self.assertTrue(fdm.load_model("ball"), "Failed to load ball aircraft model")

        # Verify basic properties exist even for simple models
        self.assertIsNotNone(fdm["position/h-sl-ft"], "Altitude property not initialized")
        self.assertIsNotNone(
            fdm["velocities/vc-kts"], "Calibrated airspeed property not initialized"
        )
        self.assertIsNotNone(fdm["attitude/phi-deg"], "Roll angle property not initialized")

        # Ball should have mass properties
        mass = fdm["inertia/weight-lbs"]
        self.assertIsNotNone(mass, "Ball weight not loaded")
        self.assertGreater(mass, 0.0, "Ball weight must be positive")

    def test_load_c172x(self):
        """
        Test loading C172X variant.

        This test verifies that different variants of the same aircraft
        family load correctly.
        """
        fdm = self.create_fdm()

        # Load the C172X model
        self.assertTrue(fdm.load_model("c172x"), "Failed to load C172X aircraft model")

        # Verify core properties
        self.assertIsNotNone(fdm["position/h-sl-ft"], "Altitude property not initialized")
        self.assertIsNotNone(
            fdm["velocities/vc-kts"], "Calibrated airspeed property not initialized"
        )

        # Verify mass properties
        mass = fdm["inertia/weight-lbs"]
        self.assertIsNotNone(mass, "Aircraft weight not loaded")
        self.assertGreater(mass, 0.0, "Weight must be positive")

    def test_multiple_aircraft_loading(self):
        """
        Test loading multiple aircraft sequentially.

        This test verifies that JSBSim can properly clean up and reinitialize
        when loading different aircraft models in sequence. This is important
        for applications that need to switch between aircraft types.
        """
        aircraft_list = ["ball", "c172p", "f16", "c172x"]

        for aircraft_name in aircraft_list:
            # Create fresh FDM for each aircraft
            fdm = self.create_fdm()

            # Load aircraft
            self.assertTrue(fdm.load_model(aircraft_name), f"Failed to load {aircraft_name}")

            # Verify basic properties for each aircraft
            self.assertIsNotNone(
                fdm["position/h-sl-ft"], f"{aircraft_name}: Altitude not initialized"
            )
            self.assertIsNotNone(
                fdm["velocities/vc-kts"], f"{aircraft_name}: Airspeed not initialized"
            )
            self.assertIsNotNone(
                fdm["attitude/phi-deg"], f"{aircraft_name}: Roll angle not initialized"
            )

            # Verify mass is reasonable
            mass = fdm["inertia/weight-lbs"]
            self.assertGreater(mass, 0.0, f"{aircraft_name}: Weight must be positive")

            # Clean up
            self.delete_fdm()

    def test_property_tree_navigation(self):
        """
        Test property tree navigation and access patterns.

        This test verifies that the property tree is correctly structured
        and that properties can be accessed through various paths. It tests
        the FGPropertyManager implementation.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Test top-level property categories exist
        # These should all be accessible even before initialization
        property_categories = [
            "position",
            "velocities",
            "attitude",
            "accelerations",
            "fcs",
            "propulsion",
            "atmosphere",
            "metrics",
            "inertia",
            "aero",
        ]

        for category in property_categories:
            # Test that we can access at least one property in each category
            if category == "position":
                self.assertIsNotNone(fdm["position/h-sl-ft"])
            elif category == "velocities":
                self.assertIsNotNone(fdm["velocities/vc-kts"])
            elif category == "attitude":
                self.assertIsNotNone(fdm["attitude/phi-deg"])
            elif category == "accelerations":
                self.assertIsNotNone(fdm["accelerations/udot-ft_sec2"])
            elif category == "fcs":
                self.assertIsNotNone(fdm["fcs/throttle-cmd-norm"])
            elif category == "propulsion":
                self.assertIsNotNone(fdm["propulsion/engine/thrust-lbs"])
            elif category == "atmosphere":
                self.assertIsNotNone(fdm["atmosphere/P-psf"])
            elif category == "metrics":
                self.assertIsNotNone(fdm["metrics/Sw-sqft"])
            elif category == "inertia":
                self.assertIsNotNone(fdm["inertia/weight-lbs"])
            elif category == "aero":
                self.assertIsNotNone(fdm["aero/alpha-rad"])

    def test_mass_properties_loaded(self):
        """
        Test that mass properties are correctly loaded from aircraft XML.

        This test verifies that the FGMassBalance component correctly parses
        and initializes mass properties including weight, CG location, and
        moments of inertia.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Verify weight is loaded
        weight = fdm["inertia/weight-lbs"]
        self.assertIsNotNone(weight, "Weight not loaded")
        self.assertGreater(weight, 0.0, "Weight must be positive")

        # Verify CG location is loaded
        cg_x = fdm["inertia/cg-x-in"]
        self.assertIsNotNone(cg_x, "CG X location not loaded")

        cg_y = fdm["inertia/cg-y-in"]
        self.assertIsNotNone(cg_y, "CG Y location not loaded")

        cg_z = fdm["inertia/cg-z-in"]
        self.assertIsNotNone(cg_z, "CG Z location not loaded")

        # Verify moments of inertia are loaded
        ixx = fdm["inertia/ixx-slugs_ft2"]
        self.assertIsNotNone(ixx, "Ixx not loaded")
        self.assertGreater(ixx, 0.0, "Ixx must be positive")

        iyy = fdm["inertia/iyy-slugs_ft2"]
        self.assertIsNotNone(iyy, "Iyy not loaded")
        self.assertGreater(iyy, 0.0, "Iyy must be positive")

        izz = fdm["inertia/izz-slugs_ft2"]
        self.assertIsNotNone(izz, "Izz not loaded")
        self.assertGreater(izz, 0.0, "Izz must be positive")

    def test_engine_configuration_loaded(self):
        """
        Test that engine configuration is correctly loaded.

        This test verifies that the FGPropulsion component correctly parses
        engine definitions and initializes engine properties. It tests both
        piston engines (C172) and turbine engines (F-16).
        """
        # Test piston engine (C172)
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Verify engine exists
        self.assertIsNotNone(
            fdm["propulsion/engine/thrust-lbs"], "C172 engine thrust property not initialized"
        )
        self.assertIsNotNone(
            fdm["propulsion/engine/power-hp"], "C172 engine power property not initialized"
        )

        # Verify propeller exists for piston engine
        self.assertIsNotNone(
            fdm["propulsion/engine/propeller-rpm"], "C172 propeller RPM not initialized"
        )

        self.delete_fdm()

        # Test turbine engine (F-16)
        fdm = self.create_fdm()
        fdm.load_model("f16")

        # Verify turbine engine properties
        self.assertIsNotNone(
            fdm["propulsion/engine/thrust-lbs"], "F-16 engine thrust property not initialized"
        )

        # F-16 uses N1/N2 for turbine engine
        # Note: Property names may vary by engine definition
        # Just verify engine properties exist
        thrust = fdm["propulsion/engine/thrust-lbs"]
        self.assertIsNotNone(thrust, "F-16 thrust property not initialized")

    def test_aerodynamic_reference_values(self):
        """
        Test that aerodynamic reference values are correctly loaded.

        This test verifies that the FGAerodynamics component has access to
        the correct reference values for aerodynamic calculations (wing area,
        span, chord, etc.).
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Wing reference area
        wing_area = fdm["metrics/Sw-sqft"]
        self.assertIsNotNone(wing_area, "Wing area not loaded")
        self.assertGreater(wing_area, 0.0, "Wing area must be positive")
        # C172 has approximately 174 sq ft wing area
        self.assertGreater(wing_area, 100.0, "C172 wing area too small")
        self.assertLess(wing_area, 300.0, "C172 wing area too large")

        # Wing span
        wing_span = fdm["metrics/bw-ft"]
        self.assertIsNotNone(wing_span, "Wing span not loaded")
        self.assertGreater(wing_span, 0.0, "Wing span must be positive")
        # C172 has approximately 36 ft wing span
        self.assertGreater(wing_span, 30.0, "C172 wing span too small")
        self.assertLess(wing_span, 50.0, "C172 wing span too large")

        # Mean aerodynamic chord
        wing_chord = fdm["metrics/cbarw-ft"]
        self.assertIsNotNone(wing_chord, "Wing chord not loaded")
        self.assertGreater(wing_chord, 0.0, "Wing chord must be positive")

        # Reference point locations
        aero_rp_x = fdm["metrics/aero-rp-x-in"]
        self.assertIsNotNone(aero_rp_x, "Aerodynamic reference point X not loaded")

        aero_rp_y = fdm["metrics/aero-rp-y-in"]
        self.assertIsNotNone(aero_rp_y, "Aerodynamic reference point Y not loaded")

        aero_rp_z = fdm["metrics/aero-rp-z-in"]
        self.assertIsNotNone(aero_rp_z, "Aerodynamic reference point Z not loaded")

    def test_initial_conditions_system(self):
        """
        Test that initial conditions can be set and retrieved.

        This test verifies that the FGInitialCondition component works
        correctly and that initial conditions propagate through the system.
        """
        fdm = self.create_fdm()
        fdm.load_model("c172p")

        # Set initial altitude
        test_altitude = 5000.0
        fdm["ic/h-sl-ft"] = test_altitude

        # Set initial airspeed
        test_airspeed = 100.0
        fdm["ic/vc-kts"] = test_airspeed

        # Set initial heading
        test_heading = 270.0
        fdm["ic/psi-true-deg"] = test_heading

        # Initialize
        self.assertTrue(fdm.run_ic(), "Failed to initialize with custom initial conditions")

        # Verify initial conditions were applied
        altitude = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(
            altitude, test_altitude, delta=1.0, msg="Initial altitude not set correctly"
        )

        airspeed = fdm["velocities/vc-kts"]
        self.assertAlmostEqual(
            airspeed, test_airspeed, delta=1.0, msg="Initial airspeed not set correctly"
        )

        heading = fdm["attitude/psi-deg"]
        self.assertAlmostEqual(
            heading, test_heading, delta=1.0, msg="Initial heading not set correctly"
        )

    def test_xml_parsing_validation(self):
        """
        Test XML parsing correctness.

        This test verifies that aircraft XML files are correctly parsed
        and that all required elements are present. It tests the
        FGXMLElement implementation.
        """
        fdm = self.create_fdm()

        # Load a well-formed aircraft
        self.assertTrue(fdm.load_model("c172p"), "Failed to load well-formed aircraft XML")

        # Verify that after loading, all expected properties exist
        # This indirectly tests that XML parsing was successful
        expected_properties = [
            "metrics/Sw-sqft",
            "metrics/bw-ft",
            "metrics/cbarw-ft",
            "inertia/weight-lbs",
            "inertia/ixx-slugs_ft2",
            "inertia/iyy-slugs_ft2",
            "inertia/izz-slugs_ft2",
        ]

        for prop in expected_properties:
            value = fdm[prop]
            self.assertIsNotNone(value, f"Expected property {prop} not found after XML parsing")
            self.assertGreater(value, 0.0, f"Property {prop} has invalid value after parsing")


if __name__ == "__main__":
    RunTest(TestAircraftLoading)
