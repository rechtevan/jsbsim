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


if __name__ == "__main__":
    unittest.main()
