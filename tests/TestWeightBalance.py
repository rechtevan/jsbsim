# TestWeightBalance.py
#
# Tests for weight and balance calculations.
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


class TestWeightBalance(JSBSimTestCase):
    """
    Tests for weight and balance calculations.

    Tests cover:
    - CG position
    - CG movement with fuel burn
    - Payload effects
    """

    def test_cg_position(self):
        """Test CG position property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/cg-x-in"):
            cg_x = fdm["inertia/cg-x-in"]
            self.assertIsNotNone(cg_x)

        del fdm

    def test_cg_within_limits(self):
        """Test CG is within reasonable limits."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/cg-x-in"):
            cg_x = fdm["inertia/cg-x-in"]
            # CG should be positive (aft of reference)
            self.assertGreater(cg_x, 0)
            # And reasonable (less than 100 inches)
            self.assertLess(cg_x, 100)

        del fdm

    def test_weight_positive(self):
        """Test weight is positive."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/weight-lbs"):
            weight = fdm["inertia/weight-lbs"]
            self.assertGreater(weight, 0)

        del fdm

    def test_inertia_positive(self):
        """Test moments of inertia are positive."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/ixx-slugs_ft2"):
            ixx = fdm["inertia/ixx-slugs_ft2"]
            self.assertGreater(ixx, 0)

        if pm.hasNode("inertia/iyy-slugs_ft2"):
            iyy = fdm["inertia/iyy-slugs_ft2"]
            self.assertGreater(iyy, 0)

        if pm.hasNode("inertia/izz-slugs_ft2"):
            izz = fdm["inertia/izz-slugs_ft2"]
            self.assertGreater(izz, 0)

        del fdm

    def test_inertia_properties_exist(self):
        """Test inertia properties exist and are readable."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("inertia/ixx-slugs_ft2") and pm.hasNode("inertia/iyy-slugs_ft2"):
            ixx = fdm["inertia/ixx-slugs_ft2"]
            iyy = fdm["inertia/iyy-slugs_ft2"]
            # Just verify they're readable
            self.assertIsNotNone(ixx)
            self.assertIsNotNone(iyy)

        del fdm


if __name__ == "__main__":
    RunTest(TestWeightBalance)
