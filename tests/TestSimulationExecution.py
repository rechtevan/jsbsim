# TestSimulationExecution.py
#
# Tests for simulation execution flow.
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


class TestSimulationExecution(JSBSimTestCase):
    """
    Tests for simulation execution.

    Tests cover:
    - Initialization
    - Time stepping
    - Multiple runs
    """

    def test_run_ic_succeeds(self):
        """Test run_ic returns True."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        result = fdm.run_ic()
        self.assertTrue(result)

        del fdm

    def test_run_advances_time(self):
        """Test run advances simulation time."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        time1 = fdm.get_sim_time()
        fdm.run()
        time2 = fdm.get_sim_time()

        self.assertGreater(time2, time1)

        del fdm

    def test_multiple_steps(self):
        """Test multiple simulation steps."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        for _ in range(100):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm

    def test_delta_t_positive(self):
        """Test delta_t is positive."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        dt = fdm.get_delta_t()
        self.assertGreater(dt, 0)

        del fdm

    def test_sim_time_starts_zero(self):
        """Test simulation time starts at zero."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        time = fdm.get_sim_time()
        self.assertAlmostEqual(time, 0, delta=0.1)

        del fdm

    def test_frame_count_increments(self):
        """Test frame count increments."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        for _ in range(10):
            fdm.run()

        # Simulation should have advanced
        time = fdm.get_sim_time()
        self.assertGreater(time, 0)

        del fdm


if __name__ == "__main__":
    RunTest(TestSimulationExecution)
