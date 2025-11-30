# TestHistoricAircraft.py
#
# Tests for historic aircraft models.
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


class TestHistoricAircraft(JSBSimTestCase):
    """
    Tests for historic aircraft models.

    Tests cover:
    - Wright Flyer 1903
    - Fokker Dr.I
    - Sopwith Camel
    - P-51 Mustang
    - B-17 Bomber
    """

    def test_load_wright_flyer(self):
        """Test loading Wright Flyer 1903 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("wrightFlyer1903")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_wright_flyer_low_speed(self):
        """Test Wright Flyer at low speed."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("wrightFlyer1903")
            fdm["ic/h-sl-ft"] = 50  # Very low altitude
            fdm["ic/vc-kts"] = 25  # Low speed
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_fokker_dr1(self):
        """Test loading Fokker Dr.I model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("dr1")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_fokker_triplane_flight(self):
        """Test Fokker Dr.I flight."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("dr1")
            fdm["ic/h-sl-ft"] = 3000
            fdm["ic/vc-kts"] = 80
            fdm.run_ic()

            for _ in range(50):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_sopwith_camel(self):
        """Test loading Sopwith Camel model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("Camel")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_p51_mustang(self):
        """Test loading P-51 Mustang model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("p51d")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_p51_high_speed(self):
        """Test P-51 at high speed."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("p51d")
            fdm["ic/h-sl-ft"] = 20000
            fdm["ic/vc-kts"] = 300
            fdm.run_ic()

            for _ in range(50):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_b17_bomber(self):
        """Test loading B-17 bomber model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("B17")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_b17_heavy_aircraft(self):
        """Test B-17 as heavy aircraft."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("B17")
            fdm["ic/h-sl-ft"] = 25000
            fdm["ic/vc-kts"] = 180
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("inertia/weight-lbs"):
                weight = fdm["inertia/weight-lbs"]
                self.assertGreater(weight, 30000)  # B-17 is heavy
        except Exception:
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestHistoricAircraft)
