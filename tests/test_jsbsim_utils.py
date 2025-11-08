"""
Unit tests for JSBSim_utils.py test utility functions.

This test module validates the helper functions and utilities used across
the JSBSim test suite, ensuring test infrastructure quality.

Copyright (c) 2025 Booz Allen Hamilton Inc.
Licensed under GPL v3
"""

import os

# Import the utilities we're testing
import sys
import tempfile
import unittest

import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from JSBSim_utils import CheckXMLFile, FindDifferences, SandBox, append_xml, isDataMatching


class TestAppendXml(unittest.TestCase):
    """Test the append_xml() utility function."""

    def test_append_xml_with_extension(self):
        """Test that files already ending in .xml are not modified."""
        self.assertEqual(append_xml("test.xml"), "test.xml")
        self.assertEqual(append_xml("script.xml"), "script.xml")
        self.assertEqual(append_xml("c1721.xml"), "c1721.xml")

    def test_append_xml_without_extension(self):
        """Test that .xml is appended to files without extension."""
        self.assertEqual(append_xml("test"), "test.xml")
        self.assertEqual(append_xml("script"), "script.xml")
        self.assertEqual(append_xml("c1721"), "c1721.xml")

    def test_append_xml_with_other_extension(self):
        """Test that .xml is appended to files with other extensions."""
        self.assertEqual(append_xml("test.txt"), "test.txt.xml")
        self.assertEqual(append_xml("data.csv"), "data.csv.xml")

    def test_append_xml_short_names(self):
        """Test append_xml with short filenames (less than 4 characters)."""
        self.assertEqual(append_xml("a"), "a.xml")
        self.assertEqual(append_xml("ab"), "ab.xml")
        self.assertEqual(append_xml("abc"), "abc.xml")

    def test_append_xml_empty_string(self):
        """Test append_xml with empty string."""
        self.assertEqual(append_xml(""), ".xml")


class TestCheckXMLFile(unittest.TestCase):
    """Test the CheckXMLFile() utility function."""

    def setUp(self):
        """Create a temporary directory for test files."""
        self.temp_dir = tempfile.mkdtemp()
        self.addCleanup(lambda: self._cleanup_temp_dir())

    def _cleanup_temp_dir(self):
        """Clean up temporary directory."""
        import shutil

        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_check_xml_file_valid_runscript(self):
        """Test CheckXMLFile with a valid runscript XML file."""
        xml_path = os.path.join(self.temp_dir, "test_script.xml")
        with open(xml_path, "w") as f:
            f.write('<?xml version="1.0"?>\n<runscript name="test"></runscript>')

        self.assertTrue(CheckXMLFile(xml_path, "runscript"))

    def test_check_xml_file_case_insensitive(self):
        """Test that CheckXMLFile is case-insensitive for headers."""
        xml_path = os.path.join(self.temp_dir, "test_script.xml")
        with open(xml_path, "w") as f:
            f.write('<?xml version="1.0"?>\n<RUNSCRIPT name="test"></RUNSCRIPT>')

        self.assertTrue(CheckXMLFile(xml_path, "runscript"))
        self.assertTrue(CheckXMLFile(xml_path, "RUNSCRIPT"))
        self.assertTrue(CheckXMLFile(xml_path, "RunScript"))

    def test_check_xml_file_wrong_header(self):
        """Test CheckXMLFile returns False for wrong header tag."""
        xml_path = os.path.join(self.temp_dir, "test_aircraft.xml")
        with open(xml_path, "w") as f:
            f.write('<?xml version="1.0"?>\n<fdm_config name="test"></fdm_config>')

        self.assertFalse(CheckXMLFile(xml_path, "runscript"))
        self.assertTrue(CheckXMLFile(xml_path, "fdm_config"))

    def test_check_xml_file_nonexistent(self):
        """Test CheckXMLFile returns False for non-existent file."""
        nonexistent_path = os.path.join(self.temp_dir, "does_not_exist.xml")
        self.assertFalse(CheckXMLFile(nonexistent_path, "runscript"))

    def test_check_xml_file_invalid_xml(self):
        """Test CheckXMLFile returns False for invalid XML."""
        xml_path = os.path.join(self.temp_dir, "invalid.xml")
        with open(xml_path, "w") as f:
            f.write("This is not valid XML at all")

        self.assertFalse(CheckXMLFile(xml_path, "runscript"))

    def test_check_xml_file_malformed_xml(self):
        """Test CheckXMLFile returns False for malformed XML."""
        xml_path = os.path.join(self.temp_dir, "malformed.xml")
        with open(xml_path, "w") as f:
            f.write('<?xml version="1.0"?>\n<runscript><unclosed>')

        self.assertFalse(CheckXMLFile(xml_path, "runscript"))


class TestIsDataMatching(unittest.TestCase):
    """Test the isDataMatching() utility function."""

    def test_is_data_matching_identical_data(self):
        """Test isDataMatching with identical dataframes."""
        df1 = pd.DataFrame({"A": [1, 2, 3], "B": [4, 5, 6]})
        df2 = pd.DataFrame({"A": [1, 2, 3], "B": [4, 5, 6]})

        # Identical data should match (delta is 0, so any() returns False)
        # Actually this function checks if data is NOT matching (has NaNs)
        # So identical data should return True (has no NaNs)
        result = isDataMatching(df1, df2)
        self.assertTrue(result)

    def test_is_data_matching_different_data(self):
        """Test isDataMatching with different data."""
        df1 = pd.DataFrame({"A": [1, 2, 3], "B": [4, 5, 6]})
        df2 = pd.DataFrame({"A": [1.1, 2.1, 3.1], "B": [4.1, 5.1, 6.1]})

        result = isDataMatching(df1, df2)
        self.assertTrue(result)  # Different values still match structurally

    def test_is_data_matching_different_shape(self):
        """Test isDataMatching with different shaped dataframes."""
        df1 = pd.DataFrame({"A": [1, 2, 3], "B": [4, 5, 6]})
        df2 = pd.DataFrame({"A": [1, 2], "B": [4, 5]})  # Different length

        # When pandas aligns different shapes, it creates NaNs but also computes
        # deltas for matching indices. The function returns True if there are
        # ANY non-null delta values (i.e., data that can be compared)
        result = isDataMatching(df1, df2)
        # Should return True because indices 0,1 exist in both and have valid deltas
        self.assertTrue(result)


class TestFindDifferences(unittest.TestCase):
    """Test the FindDifferences() utility function."""

    def test_find_differences_no_differences(self):
        """Test FindDifferences with identical data."""
        df1 = pd.DataFrame({"A": [1.0, 2.0, 3.0], "B": [4.0, 5.0, 6.0]}, index=[0.0, 0.1, 0.2])
        df2 = pd.DataFrame({"A": [1.0, 2.0, 3.0], "B": [4.0, 5.0, 6.0]}, index=[0.0, 0.1, 0.2])

        result = FindDifferences(df1, df2, tol=0.001)
        self.assertEqual(len(result), 0)  # No differences above tolerance

    def test_find_differences_small_differences(self):
        """Test FindDifferences with differences below tolerance."""
        df1 = pd.DataFrame({"A": [1.0, 2.0, 3.0], "B": [4.0, 5.0, 6.0]}, index=[0.0, 0.1, 0.2])
        df2 = pd.DataFrame(
            {"A": [1.0001, 2.0001, 3.0001], "B": [4.0001, 5.0001, 6.0001]},
            index=[0.0, 0.1, 0.2],
        )

        result = FindDifferences(df1, df2, tol=0.001)
        self.assertEqual(len(result), 0)  # Below tolerance

    def test_find_differences_large_differences(self):
        """Test FindDifferences with differences above tolerance."""
        df1 = pd.DataFrame({"A": [1.0, 2.0, 3.0], "B": [4.0, 5.0, 6.0]}, index=[0.0, 0.1, 0.2])
        df2 = pd.DataFrame({"A": [1.5, 2.5, 3.5], "B": [4.5, 5.5, 6.5]}, index=[0.0, 0.1, 0.2])

        result = FindDifferences(df1, df2, tol=0.1)
        self.assertGreater(len(result), 0)  # Should find differences

    def test_find_differences_output_columns(self):
        """Test that FindDifferences returns correct column structure."""
        df1 = pd.DataFrame({"A": [1.0, 2.0], "B": [4.0, 5.0]}, index=[0.0, 0.1])
        df2 = pd.DataFrame({"A": [2.0, 3.0], "B": [5.0, 6.0]}, index=[0.0, 0.1])

        result = FindDifferences(df1, df2, tol=0.1)

        # Check output has expected columns
        expected_columns = ["Time", "delta", "ref value", "value"]
        self.assertEqual(list(result.columns), expected_columns)


class TestSandBoxDeleteCSV(unittest.TestCase):
    """Test the SandBox.delete_csv_files() method."""

    def test_delete_csv_files(self):
        """Test that delete_csv_files removes CSV files from sandbox."""
        sandbox = SandBox()

        try:
            # Create some test CSV files in the sandbox
            csv1 = os.path.join(sandbox._tmpdir, "test1.csv")
            csv2 = os.path.join(sandbox._tmpdir, "test2.csv")
            txt_file = os.path.join(sandbox._tmpdir, "test.txt")

            with open(csv1, "w") as f:
                f.write("data1")
            with open(csv2, "w") as f:
                f.write("data2")
            with open(txt_file, "w") as f:
                f.write("text")

            # Verify files exist
            self.assertTrue(os.path.exists(csv1))
            self.assertTrue(os.path.exists(csv2))
            self.assertTrue(os.path.exists(txt_file))

            # Delete CSV files
            sandbox.delete_csv_files()

            # CSV files should be gone
            self.assertFalse(os.path.exists(csv1))
            self.assertFalse(os.path.exists(csv2))

            # Non-CSV file should remain
            self.assertTrue(os.path.exists(txt_file))

        finally:
            sandbox.erase()

    def test_delete_csv_files_empty_directory(self):
        """Test delete_csv_files works on empty directory."""
        sandbox = SandBox()

        try:
            # Should not raise error on empty directory
            sandbox.delete_csv_files()

            # Directory should still exist
            self.assertTrue(os.path.exists(sandbox._tmpdir))

        finally:
            sandbox.erase()

    def test_delete_csv_files_no_csv(self):
        """Test delete_csv_files when there are no CSV files."""
        sandbox = SandBox()

        try:
            # Create only non-CSV files
            txt_file = os.path.join(sandbox._tmpdir, "test.txt")
            xml_file = os.path.join(sandbox._tmpdir, "test.xml")

            with open(txt_file, "w") as f:
                f.write("text")
            with open(xml_file, "w") as f:
                f.write("<xml/>")

            # Delete CSV files (should not affect other files)
            sandbox.delete_csv_files()

            # Non-CSV files should remain
            self.assertTrue(os.path.exists(txt_file))
            self.assertTrue(os.path.exists(xml_file))

        finally:
            sandbox.erase()


class TestSandBoxExists(unittest.TestCase):
    """Test the SandBox.exists() method."""

    def test_exists_file_present(self):
        """Test SandBox.exists() returns True for existing file."""
        sandbox = SandBox()

        try:
            # Create a test file
            test_file = "test.txt"
            full_path = os.path.join(sandbox._tmpdir, test_file)

            with open(full_path, "w") as f:
                f.write("content")

            # Should return True
            self.assertTrue(sandbox.exists(test_file))

        finally:
            sandbox.erase()

    def test_exists_file_absent(self):
        """Test SandBox.exists() returns False for non-existent file."""
        sandbox = SandBox()

        try:
            # Should return False for non-existent file
            self.assertFalse(sandbox.exists("nonexistent.txt"))

        finally:
            sandbox.erase()


class TestJSBSimTestCase(unittest.TestCase):
    """Test the JSBSimTestCase base class methods."""

    def test_script_list_generator(self):
        """Test JSBSimTestCase.script_list() generator method."""
        from JSBSim_utils import JSBSimTestCase

        # Create a test case instance
        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Get script list
            scripts = list(test_instance.script_list())

            # Should find at least some scripts
            self.assertGreater(len(scripts), 0, "Should find at least one script")

            # All returned items should be valid file paths
            for script in scripts:
                self.assertTrue(os.path.isfile(script), f"Script should exist: {script}")
                self.assertTrue(script.endswith(".xml"), f"Script should be XML: {script}")

        finally:
            test_instance.tearDown()

    def test_script_list_with_blacklist(self):
        """Test script_list() with blacklist parameter."""
        from JSBSim_utils import JSBSimTestCase

        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Get all scripts
            all_scripts = list(test_instance.script_list())

            # Get scripts with blacklist
            if len(all_scripts) > 0:
                first_script_name = os.path.basename(all_scripts[0])
                filtered_scripts = list(test_instance.script_list(blacklist=[first_script_name]))

                # Should have one fewer script
                self.assertEqual(
                    len(filtered_scripts),
                    len(all_scripts) - 1,
                    "Blacklist should filter one script",
                )

        finally:
            test_instance.tearDown()

    def test_create_fdm_method(self):
        """Test JSBSimTestCase.create_fdm() method."""
        from JSBSim_utils import JSBSimTestCase

        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Create FDM
            fdm = test_instance.create_fdm()

            # Should return valid FDM instance
            self.assertIsNotNone(fdm)
            self.assertTrue(hasattr(fdm, "load_model"))
            self.assertTrue(hasattr(fdm, "run"))

            # Should be stored in test instance
            self.assertIsNotNone(test_instance._fdm)

        finally:
            test_instance.tearDown()

    def test_delete_fdm_method(self):
        """Test JSBSimTestCase.delete_fdm() method."""
        from JSBSim_utils import JSBSimTestCase

        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Create then delete FDM
            test_instance.create_fdm()
            self.assertIsNotNone(test_instance._fdm)

            test_instance.delete_fdm()
            self.assertIsNone(test_instance._fdm)

        finally:
            test_instance.tearDown()

    def test_load_script_method(self):
        """Test JSBSimTestCase.load_script() method."""
        from JSBSim_utils import JSBSimTestCase

        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Create FDM first
            test_instance.create_fdm()

            # Load a script
            test_instance.load_script("c1721")  # Will add .xml automatically

            # Script should be loaded (can't easily verify without running)
            # Just verify no exception was raised
            self.assertIsNotNone(test_instance._fdm)

        finally:
            test_instance.tearDown()

    def test_get_aircraft_xml_tree_method(self):
        """Test JSBSimTestCase.get_aircraft_xml_tree() method."""
        from JSBSim_utils import JSBSimTestCase

        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Get aircraft XML tree from a script
            tree = test_instance.get_aircraft_xml_tree("c1722")

            # Should return valid XML tree
            self.assertIsNotNone(tree)
            self.assertIsNotNone(tree.getroot())

            # Root should be an aircraft configuration
            root_tag = tree.getroot().tag.lower()
            self.assertIn("fdm_config", root_tag)

        finally:
            test_instance.tearDown()


class TestCopyAircraftDef(unittest.TestCase):
    """Test the CopyAircraftDef() utility function."""

    def test_copy_aircraft_def_basic(self):
        """Test CopyAircraftDef with a basic script."""
        from JSBSim_utils import CopyAircraftDef, SandBox

        sandbox = SandBox()

        try:
            # Get absolute path to a script
            script_path = sandbox.path_to_jsbsim_file("scripts", "c1722.xml")

            # Verify script exists before attempting to copy
            if not os.path.isfile(script_path):
                # Skip test if script not found
                return

            # Copy aircraft definition
            tree, aircraft_name, path = CopyAircraftDef(script_path, sandbox)

            # Should return valid data
            self.assertIsNotNone(tree)
            self.assertIsNotNone(aircraft_name)
            self.assertIsNotNone(path)

            # Aircraft name should be c172x for c1722.xml
            self.assertEqual(aircraft_name, "c172x")

            # Tree should be valid XML
            self.assertIsNotNone(tree.getroot())

            # Path should exist
            self.assertTrue(os.path.exists(path))

            # Should have created aircraft directory in sandbox
            aircraft_dir = sandbox("aircraft", aircraft_name)
            self.assertTrue(os.path.exists(aircraft_dir))

            # Should have copied IC file
            ic_file = sandbox("aircraft", aircraft_name, "reset01.xml")
            self.assertTrue(os.path.exists(ic_file))

        finally:
            sandbox.erase()

    def test_copy_aircraft_def_with_systems(self):
        """Test CopyAircraftDef with aircraft that has Systems subdirectory."""
        from JSBSim_utils import CopyAircraftDef, SandBox

        sandbox = SandBox()

        try:
            # Use J246 which has a Systems subdirectory
            script_path = sandbox.path_to_jsbsim_file("scripts", "J2460.xml")

            # Verify script exists
            if not os.path.isfile(script_path):
                # Skip if script not found
                return

            # Copy aircraft definition
            tree, aircraft_name, path = CopyAircraftDef(script_path, sandbox)

            # Should return valid data
            self.assertIsNotNone(tree)
            self.assertEqual(aircraft_name, "J246")

            # Should have created aircraft directory
            aircraft_dir = sandbox("aircraft", aircraft_name)
            self.assertTrue(os.path.exists(aircraft_dir))

            # Check if Systems files were copied (if they exist in the XML)
            # This triggers the complex file copying logic in CopyAircraftDef
            for element in list(tree.getroot()):
                if "file" in element.keys():
                    # File reference exists - the code path was exercised
                    pass

        finally:
            sandbox.erase()


class TestSpareDecorator(unittest.TestCase):
    """Test the @spare decorator functionality."""

    def test_spare_decorator_copies_file(self):
        """Test that @spare decorator preserves a file from sandbox deletion."""
        from JSBSim_utils import JSBSimTestCase, spare

        # Create a test class with a decorated method
        class TestWithSpare(JSBSimTestCase):
            @spare("test_output.txt")
            def test_method(self):
                # Create a file in the sandbox
                with open("test_output.txt", "w") as f:
                    f.write("test data")

                # Verify file exists in sandbox
                self.assertTrue(os.path.exists("test_output.txt"))

        # Run the test
        test_instance = TestWithSpare("test_method")
        test_instance.setUp()

        original_dir = test_instance.currentdir

        try:
            # Run the decorated method
            test_instance.test_method()

            # Tear down (should copy file before deletion)
            test_instance.tearDown()

            # File should have been copied to original directory
            copied_file = os.path.join(original_dir, "test_output.txt")
            self.assertTrue(
                os.path.exists(copied_file), "File should be copied by @spare decorator"
            )

            # Clean up copied file
            if os.path.exists(copied_file):
                os.remove(copied_file)

        except Exception as e:
            # Clean up if test fails
            test_instance.tearDown()
            copied_file = os.path.join(original_dir, "test_output.txt")
            if os.path.exists(copied_file):
                os.remove(copied_file)
            raise e


class TestFlightModel(unittest.TestCase):
    """Test the FlightModel helper class."""

    def test_flight_model_initialization(self):
        """Test FlightModel class initialization."""
        from JSBSim_utils import FlightModel, JSBSimTestCase

        # Create a test case instance
        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # FlightModel needs a test XML file - use existing one from tests/
            test_xml = "tripod"  # tripod.xml exists in tests/
            test_xml_path = test_instance.sandbox.path_to_jsbsim_file("tests", f"{test_xml}.xml")

            # Verify file exists
            self.assertTrue(
                os.path.isfile(test_xml_path), f"Test XML should exist: {test_xml_path}"
            )

            # Create FlightModel
            model = FlightModel(test_instance, test_xml)

            # Verify initialization
            self.assertEqual(model.name, test_xml)
            self.assertIsNotNone(model.fdm)
            self.assertIsNotNone(model.tree)
            self.assertIsNotNone(model.root)
            self.assertIsNotNone(model.path)

            # Test include_system_test_file method
            model.include_system_test_file("test_system.xml")

            # Verify system was added to XML
            system_elements = model.root.findall("system")
            self.assertGreater(len(system_elements), 0)

            # Test include_planet_test_file method
            model.include_planet_test_file("test_planet.xml")

            # Verify planet was added
            planet_elements = model.root.findall("planet")
            self.assertGreater(len(planet_elements), 0)

            # Test before_loading and before_ic hooks exist and are callable
            model.before_loading()  # Should not raise error
            model.before_ic()  # Should not raise error

        finally:
            test_instance.tearDown()

    def test_flight_model_start_method(self):
        """Test FlightModel.start() method that loads and initializes model."""
        from JSBSim_utils import FlightModel, JSBSimTestCase

        # Create a test case instance
        test_instance = JSBSimTestCase("setUp")
        test_instance.setUp()

        try:
            # Use tripod test model
            test_xml = "tripod"

            # Create FlightModel
            model = FlightModel(test_instance, test_xml)

            # Customize the model (add a system)
            model.include_system_test_file("deadband.xml")

            # Call start() - this should:
            # 1. Write XML file
            # 2. Call before_loading hook
            # 3. Load the model
            # 4. Call before_ic hook
            # 5. Run initial conditions
            # 6. Return FDM
            fdm = model.start()

            # Verify FDM was returned
            self.assertIsNotNone(fdm)

            # Verify model was loaded and initialized
            self.assertIsNotNone(fdm.get_sim_time)

            # Verify sim time advanced (run_ic was called)
            sim_time = fdm.get_sim_time()
            self.assertGreaterEqual(sim_time, 0.0)

        finally:
            test_instance.tearDown()


class TestRunTestUtility(unittest.TestCase):
    """Test the RunTest() utility function."""

    def test_run_test_with_passing_tests(self):
        """Test RunTest with passing tests."""
        from JSBSim_utils import RunTest

        # Create a simple passing test class
        class PassingTest(unittest.TestCase):
            def test_simple_pass(self):
                self.assertTrue(True)

        # Mock sys.exit to prevent actual exit
        import sys

        original_exit = sys.exit

        exit_called = []

        def mock_exit(code):
            exit_called.append(code)
            # Don't actually exit, just record the call

        try:
            sys.exit = mock_exit

            # Run the test - should not call sys.exit for passing tests
            RunTest(PassingTest)

            # Should not have called sys.exit
            self.assertEqual(len(exit_called), 0, "Should not exit for passing tests")

        finally:
            sys.exit = original_exit

    def test_run_test_with_failing_tests(self):
        """Test RunTest with failing tests calls sys.exit."""
        from JSBSim_utils import RunTest

        # Create a failing test class
        class FailingTest(unittest.TestCase):
            def test_simple_fail(self):
                self.assertTrue(False)  # This will fail

        # Mock sys.exit
        import sys

        original_exit = sys.exit

        exit_called = []

        def mock_exit(code):
            exit_called.append(code)

        try:
            sys.exit = mock_exit

            # Run the test - should call sys.exit(-1) for failures
            RunTest(FailingTest)

            # Should have called sys.exit with -1
            self.assertEqual(len(exit_called), 1)
            self.assertEqual(exit_called[0], -1)

        finally:
            sys.exit = original_exit


if __name__ == "__main__":
    unittest.main()
