# TestSimulationTime.py
#
# Tests for simulation time management and integration.
# Exercises time stepping, dt, and simulation time properties.
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


class TestSimulationTime(JSBSimTestCase):
    """
    Tests for simulation time management.

    Tests cover:
    - Simulation time properties
    - Time step (dt) configuration
    - Frame count tracking
    - Time advancement
    """

    def test_sim_time_starts_at_zero(self):
        """Test that simulation time starts at zero."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        sim_time = fdm["simulation/sim-time-sec"]
        self.assertEqual(sim_time, 0.0, "Sim time should start at zero")

        del fdm

    def test_sim_time_advances(self):
        """Test that simulation time advances with each step."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        initial_time = fdm["simulation/sim-time-sec"]

        for _ in range(100):
            fdm.run()

        final_time = fdm["simulation/sim-time-sec"]
        self.assertGreater(final_time, initial_time, "Time should advance")

        del fdm

    def test_dt_property(self):
        """Test time step (dt) property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        dt = fdm["simulation/dt"]
        self.assertGreater(dt, 0, "dt should be positive")
        self.assertLess(dt, 1.0, "dt should be less than 1 second")

        del fdm

    def test_frame_count(self):
        """Test frame count advances."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Run some frames
        num_frames = 50
        for _ in range(num_frames):
            fdm.run()

        frame = fdm["simulation/frame"]
        self.assertGreaterEqual(frame, num_frames, "Frame count should advance")

        del fdm

    def test_time_matches_frame_dt(self):
        """Test that time = frame * dt approximately."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        num_frames = 100
        for _ in range(num_frames):
            fdm.run()

        sim_time = fdm["simulation/sim-time-sec"]
        dt = fdm["simulation/dt"]

        # Time should be approximately num_frames * dt
        expected_time = num_frames * dt
        self.assertAlmostEqual(sim_time, expected_time, delta=dt, msg="Time should match frame*dt")

        del fdm

    def test_run_returns_true_while_running(self):
        """Test that run() returns True while simulation is active."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Run should return True while simulation is running
        result = fdm.run()
        self.assertTrue(result, "run() should return True while running")

        del fdm

    def test_channel_rate_property(self):
        """Test channel rate property if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("simulation/channel-dt"):
            channel_dt = fdm["simulation/channel-dt"]
            self.assertIsNotNone(channel_dt, "Channel dt should be accessible")

        del fdm

    def test_do_simple_trim_flag(self):
        """Test do-simple-trim property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("simulation/do_simple_trim"):
            trim_flag = fdm["simulation/do_simple_trim"]
            self.assertIsNotNone(trim_flag, "Trim flag should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestSimulationTime)
