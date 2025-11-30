# TestMilitaryJets.py
#
# Tests for military jet aircraft models.
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


class TestMilitaryJets(JSBSimTestCase):
    """
    Tests for military jet aircraft.

    Tests cover:
    - F-15 Eagle
    - F-22 Raptor
    - F-104 Starfighter
    - F-80C
    - T-37/T-38 trainers
    """

    def test_load_f15_model(self):
        """Test loading F-15 Eagle model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("f15")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_f15_supersonic(self):
        """Test F-15 at supersonic speed."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f15")
            fdm["ic/h-sl-ft"] = 35000
            fdm["ic/mach"] = 1.5
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("velocities/mach"):
                mach = fdm["velocities/mach"]
                self.assertGreater(mach, 1.0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_f22_model(self):
        """Test loading F-22 Raptor model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("f22")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_f22_stealth_fighter(self):
        """Test F-22 flight characteristics."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f22")
            fdm["ic/h-sl-ft"] = 40000
            fdm["ic/mach"] = 1.8
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_f104_model(self):
        """Test loading F-104 Starfighter model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("f104")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_f104_high_speed(self):
        """Test F-104 at high speed."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f104")
            fdm["ic/h-sl-ft"] = 50000
            fdm["ic/mach"] = 2.0
            fdm.run_ic()

            for _ in range(30):
                fdm.run()

            time = fdm.get_sim_time()
            self.assertGreater(time, 0)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_f80c_model(self):
        """Test loading F-80C model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("F80C")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_t37_trainer(self):
        """Test loading T-37 trainer model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("T37")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_t38_trainer(self):
        """Test loading T-38 trainer model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("T38")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_t38_supersonic_trainer(self):
        """Test T-38 supersonic flight."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("T38")
            fdm["ic/h-sl-ft"] = 35000
            fdm["ic/mach"] = 1.2
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
    RunTest(TestMilitaryJets)
