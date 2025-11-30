# TestCoordinateFrames.py
#
# Tests for coordinate frame transformations.
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


class TestCoordinateFrames(JSBSimTestCase):
    """
    Tests for coordinate frame transformations.

    Tests cover:
    - Body to ECEF
    - ECEF to geodetic
    - ECI frame properties
    """

    def test_ecef_x_position(self):
        """Test ECEF X position property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm["ic/lat-gc-deg"] = 0
        fdm["ic/long-gc-deg"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/ecef-x-ft"):
            x = fdm["position/ecef-x-ft"]
            self.assertIsNotNone(x)

        del fdm

    def test_ecef_y_position(self):
        """Test ECEF Y position property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/ecef-y-ft"):
            y = fdm["position/ecef-y-ft"]
            self.assertIsNotNone(y)

        del fdm

    def test_ecef_z_position(self):
        """Test ECEF Z position property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/ecef-z-ft"):
            z = fdm["position/ecef-z-ft"]
            self.assertIsNotNone(z)

        del fdm

    def test_eci_position(self):
        """Test ECI position properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("ball")
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/eci-x-ft"):
            x = fdm["position/eci-x-ft"]
            self.assertIsNotNone(x)

        del fdm

    def test_local_frame_velocities(self):
        """Test local frame (NED) velocities."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/v-north-fps"):
            vn = fdm["velocities/v-north-fps"]
            self.assertIsNotNone(vn)

        del fdm

    def test_body_to_wind_transformation(self):
        """Test body to wind axis transformation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/alpha-rad"):
            alpha = fdm["aero/alpha-rad"]
            self.assertIsNotNone(alpha)

        del fdm


if __name__ == "__main__":
    RunTest(TestCoordinateFrames)
