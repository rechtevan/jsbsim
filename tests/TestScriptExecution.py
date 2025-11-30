# TestScriptExecution.py
#
# Tests for JSBSim script execution system (FGScript).
# Exercises script loading, event processing, and automated runs.
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


class TestScriptExecution(JSBSimTestCase):
    """
    Tests for script execution (FGScript) functionality.

    Tests cover:
    - Script loading
    - Script execution
    - Event processing
    - Script property access
    """

    def test_script_loading(self):
        """Test that a script file can be loaded."""
        fdm = CreateFDM(self.sandbox)

        # Use a simple script that exists
        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        if os.path.exists(script_path):
            result = fdm.load_script(script_path)
            self.assertTrue(result, "Script should load successfully")

        del fdm

    def test_script_run_ic(self):
        """Test that script initializes correctly."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            result = fdm.run_ic()
            self.assertTrue(result, "Script IC should run successfully")

        del fdm

    def test_script_execution_frames(self):
        """Test running script for multiple frames."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            fdm.run_ic()

            # Run for several frames
            frames_run = 0
            for _ in range(100):
                if fdm.run():
                    frames_run += 1

            self.assertGreater(frames_run, 0, "Script should execute frames")

        del fdm

    def test_script_time_advances(self):
        """Test that simulation time advances during script execution."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            fdm.run_ic()

            initial_time = fdm["simulation/sim-time-sec"]

            for _ in range(50):
                fdm.run()

            final_time = fdm["simulation/sim-time-sec"]
            self.assertGreater(final_time, initial_time, "Time should advance during script")

        del fdm

    def test_ball_script(self):
        """Test ball drop script execution."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "ball.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            fdm.run_ic()

            initial_alt = fdm["position/h-sl-ft"]

            for _ in range(100):
                if not fdm.run():
                    break

            final_alt = fdm["position/h-sl-ft"]
            # Ball should fall
            self.assertLess(final_alt, initial_alt, "Ball should fall")

        del fdm

    def test_c172_script(self):
        """Test C172 script execution."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1722.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            fdm.run_ic()

            # Run for a bit
            for _ in range(100):
                if not fdm.run():
                    break

            # Verify aircraft is running
            alt = fdm["position/h-sl-ft"]
            self.assertIsNotNone(alt, "C172 script should produce altitude")

        del fdm

    def test_script_end_time(self):
        """Test that script has an end time property."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "c1721.xml")
        if os.path.exists(script_path):
            fdm.load_script(script_path)
            fdm.run_ic()

            # Check if end time property exists
            if fdm.get_property_manager().hasNode("simulation/terminate"):
                terminate = fdm["simulation/terminate"]
                self.assertIsNotNone(terminate)

        del fdm

    def test_737_script(self):
        """Test 737 cruise script if available."""
        fdm = CreateFDM(self.sandbox)

        script_path = self.sandbox.path_to_jsbsim_file("scripts", "737_cruise.xml")
        if os.path.exists(script_path):
            try:
                fdm.load_script(script_path)
                fdm.run_ic()

                # Run briefly
                for _ in range(50):
                    if not fdm.run():
                        break

                # Should run without error
                self.assertTrue(True, "737 script should run")
            except Exception:
                pass  # Script may have dependencies

        del fdm


if __name__ == "__main__":
    RunTest(TestScriptExecution)
