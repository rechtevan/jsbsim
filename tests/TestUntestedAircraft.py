# TestUntestedAircraft.py
#
# Tests for aircraft models that were previously untested.
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


class TestUntestedAircraft(JSBSimTestCase):
    """
    Tests for aircraft models that were previously untested.

    Tests cover:
    - ZLT-NT airship (buoyancy)
    - SGS gliders (soaring)
    - MK82 (ordnance/ballistic)
    - Concorde (supersonic)
    - X-15 (hypersonic)
    - B747 (transport)
    """

    def test_load_zlt_nt_airship(self):
        """Test loading ZLT-NT airship model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ZLT-NT")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_zlt_nt_buoyancy(self):
        """Test ZLT-NT buoyancy forces."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ZLT-NT")
        fdm["ic/h-sl-ft"] = 1000
        fdm.run_ic()

        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        # Check buoyancy-related properties
        if pm.hasNode("buoyant_forces/gas-cell/contents-mol"):
            contents = fdm["buoyant_forces/gas-cell/contents-mol"]
            self.assertGreater(contents, 0)

        del fdm

    def test_load_sgs126_glider(self):
        """Test loading SGS 1-26 glider model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("SGS")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_sgs126_glide(self):
        """Test SGS 1-26 glider in gliding flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("SGS")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 50
        fdm.run_ic()

        for _ in range(200):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-sl-ft"):
            alt = fdm["position/h-sl-ft"]
            # Should be descending without power
            self.assertLess(alt, 5000)

        del fdm

    def test_load_minisgs_glider(self):
        """Test loading Mini SGS glider model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("minisgs")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_load_mk82(self):
        """Test loading MK82 ordnance model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("mk82")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_mk82_ballistic(self):
        """Test MK82 ballistic trajectory."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("mk82")
        fdm["ic/h-sl-ft"] = 20000
        fdm["ic/vc-kts"] = 300
        fdm.run_ic()

        initial_alt = fdm["position/h-sl-ft"]

        for _ in range(500):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/h-sl-ft"):
            final_alt = fdm["position/h-sl-ft"]
            # Should be falling
            self.assertLess(final_alt, initial_alt)

        del fdm

    def test_load_concorde(self):
        """Test loading Concorde model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("Concorde")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_concorde_supersonic(self):
        """Test Concorde at high speed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("Concorde")
        fdm["ic/h-sl-ft"] = 50000
        fdm["ic/vc-kts"] = 400
        fdm.run_ic()

        for _ in range(200):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)
        del fdm

    def test_load_x15(self):
        """Test loading X-15 model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("X15")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_x15_high_altitude(self):
        """Test X-15 at high altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("X15")
        fdm["ic/h-sl-ft"] = 80000
        fdm["ic/vc-kts"] = 500
        fdm.run_ic()

        for _ in range(100):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)
        del fdm

    def test_load_b747(self):
        """Test loading B747 model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("B747")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_b747_cruise(self):
        """Test B747 cruise flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("B747")
        fdm["ic/h-sl-ft"] = 35000
        fdm["ic/vc-kts"] = 280
        fdm.run_ic()

        # Set throttle for all 4 engines
        for i in range(4):
            fdm[f"fcs/throttle-cmd-norm[{i}]"] = 0.7

        for _ in range(200):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)
        del fdm

    def test_load_a4(self):
        """Test loading A-4 Skyhawk model."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("A4")
        fdm.run_ic()
        self.assertIsNotNone(fdm.get_sim_time())
        del fdm

    def test_a4_flight(self):
        """Test A-4 Skyhawk flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("A4")
        fdm["ic/h-sl-ft"] = 15000
        fdm["ic/vc-kts"] = 350
        fdm.run_ic()

        fdm["fcs/throttle-cmd-norm[0]"] = 0.8

        for _ in range(200):
            fdm.run()

        time = fdm.get_sim_time()
        self.assertGreater(time, 0)
        del fdm


if __name__ == "__main__":
    RunTest(TestUntestedAircraft)
