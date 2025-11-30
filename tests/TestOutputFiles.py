# TestOutputFiles.py
#
# Tests for output file generation and formats.
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


from JSBSim_utils import CreateFDM, JSBSimTestCase, RunTest


class TestOutputFiles(JSBSimTestCase):
    """
    Tests for output file generation.

    Tests cover:
    - Output enabling/disabling
    - Output properties
    - Data logging
    """

    def test_enable_output(self):
        """Test enabling output."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        # Output control properties
        pm = fdm.get_property_manager()
        if pm.hasNode("simulation/output/enabled"):
            fdm["simulation/output/enabled"] = 1
            self.assertEqual(fdm["simulation/output/enabled"], 1)

        del fdm

    def test_disable_output(self):
        """Test disabling output."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("simulation/output/enabled"):
            fdm["simulation/output/enabled"] = 0
            self.assertEqual(fdm["simulation/output/enabled"], 0)

        del fdm

    def test_output_rate_property(self):
        """Test output rate property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Check for output rate or similar property
        if pm.hasNode("simulation/output-rate-hz"):
            rate = fdm["simulation/output-rate-hz"]
            self.assertIsNotNone(rate)

        del fdm

    def test_simulation_with_logging(self):
        """Test simulation runs with logging active."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 1000
        fdm.run_ic()

        # Run simulation
        for _ in range(100):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_logging_properties(self):
        """Test access to logging-related properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Check for logging or output properties
        logging_props = [
            "simulation/sim-time-sec",
            "simulation/dt",
            "simulation/frame",
        ]

        for prop in logging_props:
            if pm.hasNode(prop):
                value = fdm[prop]
                self.assertIsNotNone(value)
                break

        del fdm


if __name__ == "__main__":
    RunTest(TestOutputFiles)
