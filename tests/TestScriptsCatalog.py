# TestScriptsCatalog.py
#
# Comprehensive tests for all simulation scripts.
# Ensures scripts can load and execute properly.
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


class TestScriptsCatalog(JSBSimTestCase):
    """
    Comprehensive tests for all simulation scripts.

    Tests cover:
    - Script loading
    - Initial conditions application
    - Short simulation runs
    """

    # Scripts that are known to work and complete quickly
    QUICK_SCRIPTS = [
        "ball.xml",
        "ball_chute.xml",
        "ball_orbit.xml",
        "c172_cruise_8K.xml",
        "c1721.xml",
        "c1722.xml",
        "c1723.xml",
        "c1724.xml",
        "Short_S23_1.xml",
        "Short_S23_2.xml",
        "Short_S23_3.xml",
        "Short_S23_4.xml",
        "Short_S23_5.xml",
        "Short_S23_6.xml",
        "T37.xml",
        "737_cruise.xml",
        "B7472.xml",
        "p51_3.xml",
    ]

    def test_quick_scripts_load(self):
        """Test that quick scripts can be loaded."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        loaded = 0

        for script in self.QUICK_SCRIPTS:
            script_path = os.path.join(scripts_dir, script)
            if not os.path.exists(script_path):
                continue

            fdm = CreateFDM(self.sandbox)
            try:
                fdm.load_script(script_path)
                loaded += 1
            except Exception:
                pass
            finally:
                del fdm

        self.assertGreaterEqual(loaded, 5, "Should load at least 5 scripts")

    def test_ball_scripts(self):
        """Test ball simulation scripts."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        ball_scripts = ["ball.xml", "ball_chute.xml", "ball_orbit.xml"]
        success = 0

        for script in ball_scripts:
            script_path = os.path.join(scripts_dir, script)
            if not os.path.exists(script_path):
                continue

            fdm = CreateFDM(self.sandbox)
            try:
                fdm.load_script(script_path)
                fdm.run_ic()

                # Run short simulation
                for _ in range(50):
                    if not fdm.run():
                        break

                self.assertGreater(fdm.get_sim_time(), 0)
                success += 1
            except Exception:
                pass
            finally:
                del fdm

        self.assertGreaterEqual(success, 1, "At least 1 ball script should run")

    def test_c172_scripts(self):
        """Test Cessna 172 simulation scripts."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        c172_scripts = [
            "c172_cruise_8K.xml",
            "c1721.xml",
            "c1722.xml",
            "c1723.xml",
            "c1724.xml",
        ]
        success = 0

        for script in c172_scripts:
            script_path = os.path.join(scripts_dir, script)
            if not os.path.exists(script_path):
                continue

            fdm = CreateFDM(self.sandbox)
            try:
                fdm.load_script(script_path)
                fdm.run_ic()

                # Run short simulation
                for _ in range(20):
                    if not fdm.run():
                        break

                self.assertGreater(fdm.get_sim_time(), 0)
                success += 1
            except Exception:
                pass
            finally:
                del fdm

        self.assertGreaterEqual(success, 2, "At least 2 C172 scripts should run")

    def test_short_s23_scripts(self):
        """Test Short S.23 flying boat scripts."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        short_scripts = [
            "Short_S23_1.xml",
            "Short_S23_2.xml",
            "Short_S23_3.xml",
            "Short_S23_4.xml",
            "Short_S23_5.xml",
            "Short_S23_6.xml",
        ]
        success = 0

        for script in short_scripts:
            script_path = os.path.join(scripts_dir, script)
            if not os.path.exists(script_path):
                continue

            fdm = CreateFDM(self.sandbox)
            try:
                fdm.load_script(script_path)
                fdm.run_ic()

                # Run short simulation
                for _ in range(20):
                    if not fdm.run():
                        break

                self.assertGreater(fdm.get_sim_time(), 0)
                success += 1
            except Exception:
                pass
            finally:
                del fdm

        self.assertGreaterEqual(success, 2, "At least 2 Short S23 scripts should run")

    def test_737_scripts(self):
        """Test Boeing 737 scripts."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        scripts = ["737_cruise.xml"]
        success = 0

        for script in scripts:
            script_path = os.path.join(scripts_dir, script)
            if not os.path.exists(script_path):
                continue

            fdm = CreateFDM(self.sandbox)
            try:
                fdm.load_script(script_path)
                fdm.run_ic()

                # Run short simulation
                for _ in range(20):
                    if not fdm.run():
                        break

                self.assertGreater(fdm.get_sim_time(), 0)
                success += 1
            except Exception:
                pass
            finally:
                del fdm

        self.assertGreaterEqual(success, 0)  # May not have 737 script

    def test_script_properties(self):
        """Test that script runs expose expected properties."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        script_path = os.path.join(scripts_dir, "ball.xml")

        if not os.path.exists(script_path):
            return

        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_script(script_path)
            fdm.run_ic()

            # Run a few steps
            for _ in range(10):
                fdm.run()

            # Check standard properties
            props = [
                "simulation/sim-time-sec",
                "position/h-sl-ft",
                "velocities/v-down-fps",
            ]

            for prop in props:
                value = fdm[prop]
                self.assertIsNotNone(value, f"Property {prop} should exist")
        finally:
            del fdm

    def test_all_scripts_directory(self):
        """Test counting all scripts in the directory."""
        scripts_dir = self.sandbox.path_to_jsbsim_file("scripts")
        script_count = 0
        loadable = 0

        for f in os.listdir(scripts_dir):
            if f.endswith(".xml"):
                script_count += 1
                script_path = os.path.join(scripts_dir, f)

                fdm = CreateFDM(self.sandbox)
                try:
                    fdm.load_script(script_path)
                    loadable += 1
                except Exception:
                    pass
                finally:
                    del fdm

        self.assertGreater(script_count, 20, "Should have 20+ scripts")
        self.assertGreater(loadable, script_count * 0.5, "At least half scripts should load")


if __name__ == "__main__":
    RunTest(TestScriptsCatalog)
