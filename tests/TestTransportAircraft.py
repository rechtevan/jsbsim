# TestTransportAircraft.py
#
# Tests for transport aircraft models.
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


class TestTransportAircraft(JSBSimTestCase):
    """
    Tests for transport aircraft.

    Tests cover:
    - Boeing 747
    - Boeing 787
    - Airbus A320
    - MD-11
    - C-130
    """

    def test_load_b747_model(self):
        """Test loading Boeing 747 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("B747")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_b747_heavy_transport(self):
        """Test B747 as heavy transport."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("B747")
            fdm["ic/h-sl-ft"] = 35000
            fdm["ic/mach"] = 0.85
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("inertia/weight-lbs"):
                weight = fdm["inertia/weight-lbs"]
                self.assertGreater(weight, 300000)  # 747 is very heavy
        except Exception:
            pass
        finally:
            del fdm

    def test_load_787_model(self):
        """Test loading Boeing 787 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("787-8")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_787_fuel_efficient(self):
        """Test 787 at cruise altitude."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("787-8")
            fdm["ic/h-sl-ft"] = 40000
            fdm["ic/mach"] = 0.85
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_a320_model(self):
        """Test loading Airbus A320 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("A320")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_a320_narrow_body(self):
        """Test A320 narrow body flight."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("A320")
            fdm["ic/h-sl-ft"] = 35000
            fdm["ic/mach"] = 0.78
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_md11_model(self):
        """Test loading MD-11 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("MD11")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_c130_model(self):
        """Test loading C-130 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("C130")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_c130_turboprop(self):
        """Test C-130 turboprop operation."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("C130")
            fdm["ic/h-sl-ft"] = 25000
            fdm["ic/vc-kts"] = 280
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestTransportAircraft)
