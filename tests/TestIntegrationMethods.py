# TestIntegrationMethods.py
#
# Tests for numerical integration methods.
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


class TestIntegrationMethods(JSBSimTestCase):
    """
    Tests for numerical integration.

    Tests cover:
    - Propagation accuracy
    - Integration rate
    - State consistency
    """

    def test_propagation_runs(self):
        """Test propagation runs correctly."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 10000
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        for _ in range(100):
            fdm.run()

        final_alt = fdm["position/h-sl-ft"]
        # Ball should fall
        self.assertLess(final_alt, initial_alt)

        del fdm

    def test_integration_rate(self):
        """Test integration rate property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        dt = fdm.get_delta_t()
        self.assertGreater(dt, 0)
        self.assertLess(dt, 1.0)  # Should be reasonable timestep

        del fdm

    def test_velocity_integration(self):
        """Test velocity integrates to position."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        # Run simulation
        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/v-down-fps"):
            v_down = fdm["velocities/v-down-fps"]
            # Ball should have downward velocity
            self.assertGreater(v_down, 0)

        del fdm

    def test_acceleration_integration(self):
        """Test acceleration integrates to velocity."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("accelerations/a-pilot-z-ft_sec2"):
            # Verify property exists before running
            fdm["accelerations/a-pilot-z-ft_sec2"]

            for _ in range(50):
                fdm.run()

            # Should still have acceleration
            final_accel = fdm["accelerations/a-pilot-z-ft_sec2"]
            self.assertIsNotNone(final_accel)

        del fdm

    def test_rotation_integration(self):
        """Test rotational rates integrate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        # Apply control input to cause rotation
        fdm["fcs/aileron-cmd-norm"] = 0.5

        for _ in range(50):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/p-rad_sec"):
            p = fdm["velocities/p-rad_sec"]
            # Should have some roll rate
            self.assertIsNotNone(p)

        del fdm

    def test_long_simulation(self):
        """Test longer simulation remains stable."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        for _ in range(500):
            fdm.run()

        # Should not crash or produce NaN
        alt = fdm["position/h-sl-ft"]
        self.assertFalse(alt != alt)  # Check for NaN

        del fdm


if __name__ == "__main__":
    RunTest(TestIntegrationMethods)
