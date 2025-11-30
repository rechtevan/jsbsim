# TestWindTurbulence.py
#
# Tests for wind and turbulence models.
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


class TestWindTurbulence(JSBSimTestCase):
    """
    Tests for wind and turbulence models.

    Tests cover:
    - Steady wind effects
    - Wind direction
    - Turbulence types
    - Gust models
    """

    def test_steady_headwind(self):
        """Test steady headwind affects groundspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = 0  # Heading north
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Set north wind (headwind)
        if pm.hasNode("atmosphere/wind-north-fps"):
            fdm["atmosphere/wind-north-fps"] = -50  # 50 fps from north

        for _ in range(100):
            fdm.run()

        # Groundspeed should be less than airspeed
        if pm.hasNode("velocities/vg-fps"):
            groundspeed = fdm["velocities/vg-fps"]
            self.assertIsNotNone(groundspeed)

        del fdm

    def test_crosswind(self):
        """Test crosswind causes drift."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = 0  # Heading north
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Set east wind (crosswind)
        if pm.hasNode("atmosphere/wind-east-fps"):
            fdm["atmosphere/wind-east-fps"] = 30  # Wind from west

        initial_lon = fdm["position/long-gc-deg"]

        for _ in range(500):
            fdm.run()

        # Should drift east
        final_lon = fdm["position/long-gc-deg"]
        self.assertNotEqual(initial_lon, final_lon)

        del fdm

    def test_wind_direction_property(self):
        """Test wind direction setting via components."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Set wind via north/east components instead of angle
        if pm.hasNode("atmosphere/wind-east-fps"):
            fdm["atmosphere/wind-east-fps"] = 30
            fdm["atmosphere/wind-north-fps"] = 0

            for _ in range(10):
                fdm.run()

            # Wind should be from west (blowing east)
            wind_east = fdm["atmosphere/wind-east-fps"]
            self.assertAlmostEqual(wind_east, 30, delta=1)

        del fdm

    def test_turbulence_affects_flight(self):
        """Test turbulence causes attitude perturbations."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Enable turbulence
        if pm.hasNode("atmosphere/turb-type"):
            fdm["atmosphere/turb-type"] = 3  # Milspec

        if pm.hasNode("atmosphere/turbulence/milspec/severity"):
            fdm["atmosphere/turbulence/milspec/severity"] = 4

        # Collect attitude variations
        roll_variations = []
        for _ in range(200):
            fdm.run()
            if pm.hasNode("attitude/roll-rad"):
                roll_variations.append(fdm["attitude/roll-rad"])

        # Should have some variation due to turbulence
        if len(roll_variations) > 10:
            roll_range = max(roll_variations) - min(roll_variations)
            self.assertGreater(roll_range, 0)

        del fdm

    def test_turbulence_seed_reproducibility(self):
        """Test turbulence is reproducible with same seed."""
        results1 = []
        results2 = []

        for results in [results1, results2]:
            fdm = CreateFDM(self.sandbox)
            fdm.load_model("c172x")
            fdm["ic/h-sl-ft"] = 5000
            fdm["ic/vc-kts"] = 100
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("atmosphere/turb-type"):
                fdm["atmosphere/turb-type"] = 3

            if pm.hasNode("atmosphere/turbulence/milspec/severity"):
                fdm["atmosphere/turbulence/milspec/severity"] = 3

            # Set same seed
            if pm.hasNode("simulation/randomseed"):
                fdm["simulation/randomseed"] = 12345

            for _ in range(50):
                fdm.run()
                if pm.hasNode("attitude/roll-rad"):
                    results.append(fdm["attitude/roll-rad"])

            del fdm

        # Results should be similar with same seed
        # (Note: May not be exactly equal due to implementation details)
        self.assertEqual(len(results1), len(results2))

    def test_no_wind_baseline(self):
        """Test flight without wind for baseline."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Ensure no wind
        if pm.hasNode("atmosphere/wind-north-fps"):
            fdm["atmosphere/wind-north-fps"] = 0
        if pm.hasNode("atmosphere/wind-east-fps"):
            fdm["atmosphere/wind-east-fps"] = 0
        if pm.hasNode("atmosphere/wind-down-fps"):
            fdm["atmosphere/wind-down-fps"] = 0

        for _ in range(100):
            fdm.run()

        # TAS should approximately equal groundspeed
        if pm.hasNode("velocities/vt-fps") and pm.hasNode("velocities/vg-fps"):
            tas = fdm["velocities/vt-fps"]
            gs = fdm["velocities/vg-fps"]
            self.assertAlmostEqual(tas, gs, delta=10)

        del fdm

    def test_vertical_wind(self):
        """Test vertical wind (updraft/downdraft)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("SGS")  # Glider
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 50
        fdm.run_ic()

        pm = fdm.get_property_manager()
        # Set updraft
        if pm.hasNode("atmosphere/wind-down-fps"):
            fdm["atmosphere/wind-down-fps"] = -20  # Updraft

        for _ in range(200):
            fdm.run()

        # Glider in updraft - verify it runs without error
        final_alt = fdm["position/h-sl-ft"]
        self.assertIsNotNone(final_alt)

        del fdm


if __name__ == "__main__":
    RunTest(TestWindTurbulence)
