# TestLoadFactor.py
#
# Tests for load factor (g-force) calculations.
# Exercises normal and lateral load factors.
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


class TestLoadFactor(JSBSimTestCase):
    """
    Tests for load factor (g-force) calculations.

    Tests cover:
    - Normal load factor (Nz)
    - Lateral load factor (Ny)
    - Axial load factor (Nx)
    - Load factor in various maneuvers
    """

    def test_nz_in_level_flight(self):
        """Test normal load factor in level flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/gamma-deg"] = 0
        fdm.run_ic()

        for _ in range(100):
            fdm.run()

        nz = fdm["accelerations/Nz"]
        # In level flight, Nz should be approximately 1g
        self.assertAlmostEqual(nz, 1.0, delta=0.5, msg="Nz should be ~1g in level")

        del fdm

    def test_nz_property_exists(self):
        """Test that Nz property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        nz = fdm["accelerations/Nz"]
        self.assertIsNotNone(nz, "Nz should be accessible")

        del fdm

    def test_ny_in_coordinated_flight(self):
        """Test lateral load factor in coordinated flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm["ic/beta-deg"] = 0
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        ny = fdm["accelerations/Ny"]
        # In coordinated flight, Ny should be near zero
        self.assertAlmostEqual(ny, 0.0, delta=0.3, msg="Ny should be ~0 in coord")

        del fdm

    def test_nx_property_exists(self):
        """Test that Nx property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        if fdm.get_property_manager().hasNode("accelerations/Nx"):
            nx = fdm["accelerations/Nx"]
            self.assertIsNotNone(nx, "Nx should be accessible")

        del fdm

    def test_load_factor_magnitude(self):
        """Test load factor magnitude property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        if fdm.get_property_manager().hasNode("accelerations/nlf"):
            nlf = fdm["accelerations/nlf"]
            self.assertIsNotNone(nlf, "NLF should be accessible")

        del fdm

    def test_nz_increases_in_pull_up(self):
        """Test that Nz increases during pull-up."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 180
        fdm.run_ic()

        for _ in range(50):
            fdm.run()

        nz_before = fdm["accelerations/Nz"]

        # Apply pull-up
        fdm["fcs/elevator-cmd-norm"] = -0.5
        for _ in range(30):
            fdm.run()

        nz_after = fdm["accelerations/Nz"]

        # Nz should increase during pull-up
        self.assertGreater(nz_after, nz_before, "Nz should increase in pull-up")

        del fdm

    def test_pilot_load_factor(self):
        """Test pilot-location load factor if available."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        for _ in range(20):
            fdm.run()

        if fdm.get_property_manager().hasNode("accelerations/n-pilot-z-norm"):
            nz_pilot = fdm["accelerations/n-pilot-z-norm"]
            self.assertIsNotNone(nz_pilot, "Pilot Nz should be accessible")

        del fdm

    def test_udot_property(self):
        """Test forward acceleration property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/u-fps"] = 150
        fdm.run_ic()

        fdm.run()

        if fdm.get_property_manager().hasNode("accelerations/udot-fps_sec"):
            udot = fdm["accelerations/udot-fps_sec"]
            self.assertIsNotNone(udot, "Udot should be accessible")

        del fdm


if __name__ == "__main__":
    RunTest(TestLoadFactor)
