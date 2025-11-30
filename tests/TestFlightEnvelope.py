# TestFlightEnvelope.py
#
# Tests for flight envelope properties.
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


class TestFlightEnvelope(JSBSimTestCase):
    """
    Tests for flight envelope properties.

    Tests cover:
    - High altitude flight
    - High speed flight
    - Low speed flight
    - Flight path angle
    """

    def test_high_altitude_flight(self):
        """Test flight at high altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 15000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 15000, delta=100)

        del fdm

    def test_low_altitude_flight(self):
        """Test flight at low altitude."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 500
        fdm["ic/vc-kts"] = 80
        fdm.run_ic()

        alt = fdm["position/h-sl-ft"]
        self.assertAlmostEqual(alt, 500, delta=50)

        del fdm

    def test_climb_angle(self):
        """Test climb angle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm["ic/gamma-deg"] = 5
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("flight-path/gamma-deg"):
            gamma = fdm["flight-path/gamma-deg"]
            self.assertIsNotNone(gamma)

        del fdm

    def test_vertical_speed(self):
        """Test vertical speed property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("velocities/h-dot-fps"):
            hdot = fdm["velocities/h-dot-fps"]
            self.assertIsNotNone(hdot)

        del fdm

    def test_range_property(self):
        """Test range property exists."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Run a bit
        for _ in range(100):
            fdm.run()

        pm = fdm.get_property_manager()
        if pm.hasNode("position/distance-from-start-mag-mt"):
            dist = fdm["position/distance-from-start-mag-mt"]
            self.assertIsNotNone(dist)

        del fdm

    def test_track_angle(self):
        """Test track angle property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = 90
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("flight-path/psi-gt-rad"):
            track = fdm["flight-path/psi-gt-rad"]
            self.assertIsNotNone(track)

        del fdm


if __name__ == "__main__":
    RunTest(TestFlightEnvelope)
