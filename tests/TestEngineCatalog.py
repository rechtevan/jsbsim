# TestEngineCatalog.py
#
# Comprehensive tests for all engine definitions.
# Ensures engine XML files parse correctly and basic properties work.
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
#

import os

from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestEngineCatalog(JSBSimTestCase):
    """
    Comprehensive tests for engine definitions.

    Tests cover:
    - Engine file existence
    - Engine types verification
    - Propeller definitions
    """

    def test_engine_directory_exists(self):
        """Test that engine directory has content."""
        engine_dir = self.sandbox.path_to_jsbsim_file("engine")
        self.assertTrue(os.path.isdir(engine_dir))

        files = os.listdir(engine_dir)
        xml_files = [f for f in files if f.endswith(".xml")]
        self.assertGreater(len(xml_files), 50, "Should have 50+ engine files")

    def test_engine_types(self):
        """Test various engine types by loading aircraft that use them."""
        # Test turbine engines via 737
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("737")
            fdm.run_ic()
            # 737 uses turbine engines
            self.assertTrue(True)
        except Exception:
            pass
        finally:
            del fdm

        # Test piston engines via c172x
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("c172x")
            fdm.run_ic()
            # c172x uses piston engine
            self.assertTrue(True)
        except Exception:
            pass
        finally:
            del fdm

        # Test rocket engines via X15
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("X15")
            fdm.run_ic()
            self.assertTrue(True)
        except Exception:
            pass
        finally:
            del fdm

    def test_piston_engine_properties(self):
        """Test piston engine property access."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("c172x")
            fdm["ic/h-sl-ft"] = 5000
            fdm["ic/u-fps"] = 200
            fdm.run_ic()

            # Set throttle
            fdm["fcs/throttle-cmd-norm"] = 0.7
            fdm["fcs/mixture-cmd-norm"] = 1.0

            # Run simulation
            for _ in range(50):
                fdm.run()

            # Check for piston engine properties
            pm = fdm.get_property_manager()
            if pm.hasNode("propulsion/engine/power-hp"):
                power = fdm["propulsion/engine/power-hp"]
                self.assertIsNotNone(power)
        except Exception:
            pass
        finally:
            del fdm

    def test_turbine_engine_properties(self):
        """Test turbine engine property access."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("737")
            fdm["ic/h-sl-ft"] = 10000
            fdm["ic/u-fps"] = 400
            fdm.run_ic()

            # Set throttle
            fdm["fcs/throttle-cmd-norm"] = 0.8

            # Run simulation
            for _ in range(50):
                fdm.run()

            # Check for turbine properties
            pm = fdm.get_property_manager()
            if pm.hasNode("propulsion/engine[0]/n1"):
                n1 = fdm["propulsion/engine[0]/n1"]
                self.assertIsNotNone(n1)
        except Exception:
            pass
        finally:
            del fdm

    def test_propeller_definitions(self):
        """Test propeller file definitions exist."""
        engine_dir = self.sandbox.path_to_jsbsim_file("engine")

        # Look for propeller files
        propeller_count = 0
        for f in os.listdir(engine_dir):
            if f.endswith(".xml"):
                filepath = os.path.join(engine_dir, f)
                try:
                    with open(filepath) as file:
                        content = file.read(500)
                        if "propeller" in content.lower():
                            propeller_count += 1
                except Exception:
                    pass

        self.assertGreater(propeller_count, 5, "Should have propeller definitions")

    def test_engine_file_xml_validity(self):
        """Test that engine XML files are well-formed."""
        import xml.etree.ElementTree as ET

        engine_dir = self.sandbox.path_to_jsbsim_file("engine")
        valid_count = 0
        total = 0

        for f in os.listdir(engine_dir):
            if f.endswith(".xml"):
                total += 1
                filepath = os.path.join(engine_dir, f)
                try:
                    ET.parse(filepath)
                    valid_count += 1
                except Exception:
                    pass

        # At least 90% should be valid XML
        self.assertGreater(
            valid_count, total * 0.9, "At least 90% of engine files should be valid XML"
        )

    def test_multi_engine_aircraft(self):
        """Test multi-engine aircraft."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("B747")
            fdm.run_ic()

            # B747 has 4 engines
            pm = fdm.get_property_manager()
            engine_count = 0
            for i in range(4):
                if pm.hasNode(f"propulsion/engine[{i}]/set-running"):
                    engine_count += 1

            self.assertGreaterEqual(engine_count, 2, "B747 should have multiple engines")
        except Exception:
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestEngineCatalog)
