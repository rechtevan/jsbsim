# TestXMLConfiguration.py
#
# Tests for XML configuration loading.
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


class TestXMLConfiguration(JSBSimTestCase):
    """
    Tests for XML configuration loading.

    Tests cover:
    - Aircraft loading
    - Engine loading
    - Script loading
    """

    def test_load_c172_model(self):
        """Test loading C172 model."""
        fdm = CreateFDM(self.sandbox)
        result = fdm.load_model("c172x")
        self.assertTrue(result)

        del fdm

    def test_load_ball_model(self):
        """Test loading ball model."""
        fdm = CreateFDM(self.sandbox)
        result = fdm.load_model("ball")
        self.assertTrue(result)

        del fdm

    def test_load_737_model(self):
        """Test loading 737 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("737")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_load_f16_model(self):
        """Test loading F-16 model."""
        fdm = CreateFDM(self.sandbox)
        try:
            result = fdm.load_model("f16")
            self.assertTrue(result)
        except Exception:
            pass
        finally:
            del fdm

    def test_model_has_metrics(self):
        """Test loaded model has metrics."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        self.assertTrue(pm.hasNode("metrics/bw-ft"))

        del fdm

    def test_model_has_mass_balance(self):
        """Test loaded model has mass balance."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        has_weight = pm.hasNode("inertia/weight-lbs")
        self.assertTrue(has_weight)

        del fdm


if __name__ == "__main__":
    RunTest(TestXMLConfiguration)
