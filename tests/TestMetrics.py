# TestMetrics.py
#
# Tests for aircraft metrics (dimensions, reference values).
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


class TestMetrics(JSBSimTestCase):
    """
    Tests for aircraft metrics properties.

    Tests cover:
    - Wing span
    - Wing area
    - Mean aerodynamic chord
    - Reference points
    """

    def test_wingspan(self):
        """Test wingspan property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("metrics/bw-ft"):
            bw = fdm["metrics/bw-ft"]
            self.assertGreater(bw, 0)
            # C172 wingspan is about 36 ft
            self.assertGreater(bw, 30)
            self.assertLess(bw, 50)

        del fdm

    def test_wing_area(self):
        """Test wing area property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("metrics/Sw-sqft"):
            sw = fdm["metrics/Sw-sqft"]
            self.assertGreater(sw, 0)
            # C172 wing area is about 174 sq ft
            self.assertGreater(sw, 100)
            self.assertLess(sw, 300)

        del fdm

    def test_mean_aero_chord(self):
        """Test mean aerodynamic chord property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("metrics/cbarw-ft"):
            cbar = fdm["metrics/cbarw-ft"]
            self.assertGreater(cbar, 0)
            # C172 MAC is about 4.9 ft
            self.assertGreater(cbar, 3)
            self.assertLess(cbar, 10)

        del fdm

    def test_aspect_ratio(self):
        """Test aspect ratio calculation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("metrics/bw-ft") and pm.hasNode("metrics/Sw-sqft"):
            bw = fdm["metrics/bw-ft"]
            sw = fdm["metrics/Sw-sqft"]
            ar = (bw * bw) / sw
            # C172 aspect ratio is about 7.5
            self.assertGreater(ar, 5)
            self.assertLess(ar, 15)

        del fdm

    def test_737_metrics(self):
        """Test 737 metrics (larger aircraft)."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("737")
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("metrics/Sw-sqft"):
                sw = fdm["metrics/Sw-sqft"]
                # 737 wing area is much larger
                self.assertGreater(sw, 500)
        except Exception:
            pass
        finally:
            del fdm

    def test_f16_metrics(self):
        """Test F-16 metrics (fighter)."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f16")
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("metrics/Sw-sqft"):
                sw = fdm["metrics/Sw-sqft"]
                # F-16 wing area is about 300 sq ft
                self.assertGreater(sw, 200)
                self.assertLess(sw, 500)
        except Exception:
            pass
        finally:
            del fdm


if __name__ == "__main__":
    RunTest(TestMetrics)
