# TestSimulationControl.py
#
# Tests for simulation control (hold, resume, reset).
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


class TestSimulationControl(JSBSimTestCase):
    """
    Tests for simulation control functions.

    Tests cover:
    - Hold/resume
    - Reset
    - Time step control
    """

    def test_hold_simulation(self):
        """Test holding simulation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        # Run a few steps
        for _ in range(10):
            fdm.run()

        # Hold simulation
        fdm.hold()
        self.assertTrue(fdm.holding())

        del fdm

    def test_resume_simulation(self):
        """Test resuming simulation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        fdm.hold()
        self.assertTrue(fdm.holding())

        fdm.resume()
        self.assertFalse(fdm.holding())

        del fdm

    def test_reset_to_ic(self):
        """Test resetting to initial conditions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        # Run simulation
        for _ in range(100):
            fdm.run()

        # Reset
        fdm.run_ic()
        reset_alt = fdm["position/h-sl-ft"]

        # Should be back to initial
        self.assertAlmostEqual(reset_alt, initial_alt, delta=10)

        del fdm

    def test_simulation_time_advances(self):
        """Test that simulation time advances."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        time1 = fdm.get_sim_time()

        for _ in range(10):
            fdm.run()

        time2 = fdm.get_sim_time()

        self.assertGreater(time2, time1)

        del fdm

    def test_dt_property(self):
        """Test time step property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        dt = fdm.get_delta_t()
        self.assertGreater(dt, 0)
        # Typical dt is 0.00833 (120 Hz)
        self.assertLess(dt, 0.1)

        del fdm

    def test_multiple_runs(self):
        """Test multiple consecutive runs."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        times = []
        for _ in range(5):
            fdm.run()
            times.append(fdm.get_sim_time())

        # Each time should be greater than previous
        for i in range(1, len(times)):
            self.assertGreater(times[i], times[i - 1])

        del fdm

    def test_get_property_value(self):
        """Test get_property_value method."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        alt = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(alt, 5000, delta=10)

        del fdm

    def test_set_property_value(self):
        """Test set_property_value method."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        fdm.set_property_value("position/h-sl-ft", 8000)
        alt = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(alt, 8000, delta=10)

        del fdm


if __name__ == "__main__":
    RunTest(TestSimulationControl)
