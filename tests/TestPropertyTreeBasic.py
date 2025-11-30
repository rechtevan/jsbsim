# TestPropertyTreeBasic.py
#
# Tests for property tree access and management.
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


class TestPropertyTreeBasic(JSBSimTestCase):
    """
    Tests for property tree access.

    Tests cover:
    - Property reading
    - Property writing
    - Property enumeration
    - Property manager
    """

    def test_read_position_property(self):
        """Test reading position property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 5000
        fdm.run_ic()

        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 5000, delta=10)

        del fdm

    def test_write_property(self):
        """Test writing to a property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        fdm["position/h-sl-ft"] = 8000
        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 8000, delta=10)

        del fdm

    def test_property_manager_exists(self):
        """Test property manager is accessible."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        self.assertIsNotNone(pm)

        del fdm

    def test_has_node_check(self):
        """Test hasNode method."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        self.assertTrue(pm.hasNode("position/h-sl-ft"))

        del fdm

    def test_get_property_value(self):
        """Test get_property_value method."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/h-sl-ft"] = 3000
        fdm.run_ic()

        alt = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(alt, 3000, delta=10)

        del fdm

    def test_set_property_value(self):
        """Test set_property_value method."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        fdm.set_property_value("position/h-sl-ft", 6000)
        alt = fdm.get_property_value("position/h-sl-ft")
        self.assertAlmostEqual(alt, 6000, delta=10)

        del fdm


if __name__ == "__main__":
    RunTest(TestPropertyTreeBasic)
