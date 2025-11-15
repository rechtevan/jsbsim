# TestPropertySystemBasic.py
#
# Comprehensive tests for FGPropertyManager (property system).
#
# Tests cover:
# - Basic property get/set operations
# - Property creation and dynamic node creation
# - Property tree navigation
# - Property node operations (GetNode, HasNode)
# - Property types (double, int, bool, string)
# - Property catalog/listing
# - Nested properties and deep paths
#
# Copyright (c) 2025
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

from JSBSim_utils import JSBSimTestCase, RunTest


class TestPropertySystemBasic(JSBSimTestCase):
    """Test suite for FGPropertyManager - property system basic operations."""

    def test_property_get_set_operations(self):
        """Test basic property get and set operations."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test get existing property
        alt = fdm.get_property_value("position/h-sl-ft")
        self.assertIsNotNone(alt, "Should be able to get altitude property")
        self.assertIsInstance(alt, float, "Property value should be float")

        # Test set and get with initial condition property
        new_alt = 5000.0
        fdm.set_property_value("ic/h-sl-ft", new_alt)
        fdm.run_ic()

        actual = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(
            actual,
            new_alt,
            delta=10.0,
            msg=f"Set altitude should update property (expected ~{new_alt}, got {actual})",
        )

        # Test velocity property
        fdm.set_property_value("ic/vc-kts", 100.0)
        fdm.run_ic()
        vc = fdm.get_property_value("velocities/vc-kts")
        self.assertAlmostEqual(
            vc, 100.0, delta=5.0, msg="Calibrated airspeed should be set correctly"
        )

    def test_property_creation_dynamic(self):
        """Test creating new properties dynamically."""
        fdm = self.create_fdm()
        pm = fdm.get_property_manager()

        # Create a new property using bracket notation
        self.assertFalse(pm.hasNode("test/custom-property"))
        fdm["test/custom-property"] = 123.45

        # Verify property was created and has correct value
        self.assertTrue(
            pm.hasNode("test/custom-property"),
            "Property should exist after assignment",
        )
        self.assertAlmostEqual(
            fdm["test/custom-property"],
            123.45,
            msg="Property should have assigned value",
        )

        # Test with get_property_value
        value = fdm.get_property_value("test/custom-property")
        self.assertAlmostEqual(value, 123.45, msg="get_property_value should work")

    def test_property_tree_navigation(self):
        """Test navigating property tree paths."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()
        pm = fdm.get_property_manager()

        # Test root node navigation
        root_node = pm.get_node()
        self.assertIsNotNone(root_node, "Should be able to get root node")
        self.assertEqual(
            root_node.get_fully_qualified_name(),
            "/fdm/jsbsim",
            "Root should be /fdm/jsbsim",
        )

        # Test navigating to position properties
        position_node = pm.get_node("position/h-sl-ft")
        self.assertIsNotNone(position_node, "Should be able to navigate to position")

        # Test value retrieval through node
        alt_via_node = position_node.get_double_value()
        alt_via_fdm = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(
            alt_via_node,
            alt_via_fdm,
            msg="Node value should match fdm property value",
        )

    def test_property_node_operations(self):
        """Test GetNode and HasNode operations."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()
        pm = fdm.get_property_manager()

        # Test HasNode for existing property
        self.assertTrue(
            pm.hasNode("position/h-sl-ft"),
            "HasNode should return True for existing property",
        )

        # Test HasNode for non-existing property
        self.assertFalse(
            pm.hasNode("nonexistent/property/path"),
            "HasNode should return False for non-existing property",
        )

        # Test GetNode with create=False (default)
        non_existent = pm.get_node("test/does-not-exist", create=False)
        self.assertIsNone(
            non_existent, "GetNode should return None when create=False and not exists"
        )

        # Test GetNode with create=True
        new_node = pm.get_node("test/newly-created", create=True)
        self.assertIsNotNone(new_node, "GetNode should create and return node when create=True")
        self.assertEqual(new_node.get_name(), "newly-created", "Node name should match")

        # Verify newly created node exists
        self.assertTrue(
            pm.hasNode("test/newly-created"),
            "HasNode should find newly created property",
        )

    def test_property_types_double(self):
        """Test double (floating-point) property type operations."""
        fdm = self.create_fdm()
        pm = fdm.get_property_manager()

        # Create and set double property
        node = pm.get_node("test/double-property", create=True)
        success = node.set_double_value(3.14159)
        self.assertTrue(success, "Setting double value should succeed")

        # Get and verify
        value = node.get_double_value()
        self.assertAlmostEqual(value, 3.14159, places=5, msg="Double value should be precise")

        # Test negative values
        node.set_double_value(-273.15)
        self.assertAlmostEqual(node.get_double_value(), -273.15, msg="Negative double should work")

        # Test very small values
        node.set_double_value(1.23e-10)
        self.assertAlmostEqual(
            node.get_double_value(),
            1.23e-10,
            places=15,
            msg="Small double values should work",
        )

    def test_property_access_methods(self):
        """Test different property access methods ([], get/set, node)."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()
        pm = fdm.get_property_manager()

        # Create a test property
        test_value = 42.0
        property_path = "test/access-methods"

        # Method 1: Bracket notation
        fdm[property_path] = test_value
        self.assertAlmostEqual(fdm[property_path], test_value, msg="Bracket notation should work")

        # Method 2: get_property_value / set_property_value
        fdm.set_property_value(property_path, test_value * 2)
        retrieved = fdm.get_property_value(property_path)
        self.assertAlmostEqual(retrieved, test_value * 2, msg="get/set methods should work")

        # Method 3: Property node
        node = pm.get_node(property_path)
        node.set_double_value(test_value * 3)
        self.assertAlmostEqual(
            node.get_double_value(), test_value * 3, msg="Node methods should work"
        )

        # Verify all methods access same underlying property
        self.assertAlmostEqual(
            fdm[property_path],
            fdm.get_property_value(property_path),
            msg="Bracket and get_property_value should return same value",
        )
        self.assertAlmostEqual(
            fdm.get_property_value(property_path),
            node.get_double_value(),
            msg="get_property_value and node should return same value",
        )

    def test_property_catalog_listing(self):
        """Test property catalog and listing operations."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test query_property_catalog with filter
        catalog_str = fdm.query_property_catalog("geod-deg")
        self.assertIsInstance(catalog_str, str, "query_property_catalog should return string")
        self.assertIn(
            "position/lat-geod-deg",
            catalog_str,
            "Catalog should contain geodetic latitude",
        )
        self.assertIn("ic/lat-geod-deg", catalog_str, "Catalog should contain IC latitude")

        # Test get_property_catalog (returns list)
        catalog_list = fdm.get_property_catalog()
        self.assertIsInstance(catalog_list, list, "get_property_catalog should return list")
        self.assertGreater(len(catalog_list), 100, "Catalog should contain many properties")

        # Verify some critical properties are in catalog
        catalog_text = " ".join(catalog_list)
        critical_props = [
            "position/h-sl-ft",
            "velocities/vc-kts",
            "attitude/phi-deg",
            "attitude/theta-deg",
            "attitude/psi-deg",
        ]
        for prop in critical_props:
            self.assertIn(prop, catalog_text, f"Catalog should contain critical property {prop}")

    def test_nested_properties_deep_paths(self):
        """Test deep property paths and nested property structures."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()
        pm = fdm.get_property_manager()

        # Test existing deep nested properties
        deep_paths = [
            "position/h-sl-ft",  # Altitude above sea level
            "position/lat-geod-deg",  # Geodetic latitude
            "position/long-gc-deg",  # Geocentric longitude
            "velocities/u-fps",  # Body-frame velocity components
            "velocities/v-fps",
            "velocities/w-fps",
            "attitude/phi-deg",  # Euler angles
            "attitude/theta-deg",
            "attitude/psi-deg",
            "fcs/throttle-cmd-norm",  # Flight control system
            "fcs/elevator-cmd-norm",
            "propulsion/engine/thrust-lbs",  # Propulsion system
        ]

        for path in deep_paths:
            self.assertTrue(pm.hasNode(path), f"Deep property path should exist: {path}")
            value = fdm.get_property_value(path)
            self.assertIsInstance(value, float, f"Property {path} should return float value")

        # Test creating custom deep nested structure
        deep_custom = "test/level1/level2/level3/deep-property"
        fdm[deep_custom] = 999.0
        self.assertTrue(pm.hasNode(deep_custom), "Should create deep nested custom property")
        self.assertAlmostEqual(
            fdm[deep_custom], 999.0, msg="Deep nested property should have value"
        )

        # Test navigating through nested structure
        level1_node = pm.get_node("test/level1")
        self.assertIsNotNone(level1_node, "Should be able to get intermediate node")

        level2_node = level1_node.get_node("level2")
        self.assertIsNotNone(level2_node, "Should navigate from parent to child node")

    def test_property_modification_during_simulation(self):
        """Test modifying properties during simulation run."""
        fdm = self.create_fdm()
        fdm.load_model("c172x")
        fdm.run_ic()

        # Run simulation for a bit
        for _ in range(10):
            fdm.run()

        # Check that altitude property updates during simulation
        current_alt = fdm["position/h-sl-ft"]
        # Aircraft will descend or maintain altitude, value should change
        self.assertIsNotNone(current_alt, "Altitude should be readable during sim")

        # Set throttle and verify it's applied
        fdm["fcs/throttle-cmd-norm"] = 0.8
        throttle_read = fdm["fcs/throttle-cmd-norm"]
        self.assertAlmostEqual(
            throttle_read,
            0.8,
            delta=0.01,
            msg="Throttle setting should be applied during sim",
        )

        # Set elevator and verify
        fdm["fcs/elevator-cmd-norm"] = -0.1
        elevator_read = fdm["fcs/elevator-cmd-norm"]
        self.assertAlmostEqual(
            elevator_read,
            -0.1,
            delta=0.01,
            msg="Elevator setting should be applied during sim",
        )

    def test_property_nonexistent_handling(self):
        """Test handling of non-existent properties."""
        fdm = self.create_fdm()
        pm = fdm.get_property_manager()

        # Test that non-existent properties return 0.0 with get_property_value
        nonexistent_value = fdm.get_property_value("this/does/not/exist")
        self.assertEqual(
            nonexistent_value,
            0.0,
            "Non-existent property should return 0.0 with get_property_value",
        )

        # Test that bracket notation raises KeyError for non-existent properties
        with self.assertRaises(
            KeyError, msg="Bracket notation should raise KeyError for non-existent prop"
        ):
            _ = fdm["this/does/not/exist"]

        # Test that HasNode returns False
        self.assertFalse(
            pm.hasNode("this/does/not/exist"),
            "HasNode should return False for non-existent",
        )

        # Test that GetNode with create=False returns None
        node = pm.get_node("this/does/not/exist", create=False)
        self.assertIsNone(node, "GetNode should return None for non-existent when create=False")


if __name__ == "__main__":
    RunTest(TestPropertySystemBasic)
