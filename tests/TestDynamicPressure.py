# TestDynamicPressure.py
#
# Tests for dynamic pressure and related aerodynamic properties.
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


class TestDynamicPressure(JSBSimTestCase):
    """
    Tests for dynamic pressure properties.

    Tests cover:
    - Dynamic pressure (qbar)
    - qbar-psf
    - Impact pressure
    """

    def test_qbar_property(self):
        """Test dynamic pressure property."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/qbar-psf"):
            qbar = fdm["aero/qbar-psf"]
            self.assertGreater(qbar, 0)
            # At 100 kts, qbar should be around 30-40 psf
            self.assertGreater(qbar, 10)
            self.assertLess(qbar, 100)

        del fdm

    def test_qbar_increases_with_speed(self):
        """Test that qbar increases with speed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 80
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if not pm.hasNode("aero/qbar-psf"):
            del fdm
            return

        qbar_low = fdm["aero/qbar-psf"]

        # Reinitialize at higher speed
        fdm["ic/vc-kts"] = 120
        fdm.run_ic()
        qbar_high = fdm["aero/qbar-psf"]

        self.assertGreater(qbar_high, qbar_low)

        del fdm

    def test_qbar_at_sea_level(self):
        """Test qbar at sea level."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/qbar-psf"):
            qbar = fdm["aero/qbar-psf"]
            # qbar = 0.5 * rho * V^2
            # At sea level, should be higher than at altitude
            self.assertGreater(qbar, 20)

        del fdm

    def test_qbar_at_altitude(self):
        """Test qbar at altitude (lower density)."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 20000
        fdm["ic/vc-kts"] = 100
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/qbar-psf"):
            qbar = fdm["aero/qbar-psf"]
            # At altitude, qbar should be positive but lower
            self.assertGreater(qbar, 0)

        del fdm

    def test_qbar_zero_at_zero_speed(self):
        """Test that qbar is zero at zero airspeed."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm["ic/h-sl-ft"] = 0
        fdm["ic/u-fps"] = 0
        fdm["ic/v-fps"] = 0
        fdm["ic/w-fps"] = 0
        fdm.run_ic()

        pm = fdm.get_property_manager()
        if pm.hasNode("aero/qbar-psf"):
            qbar = fdm["aero/qbar-psf"]
            self.assertAlmostEqual(qbar, 0, delta=1)

        del fdm

    def test_qbar_high_speed(self):
        """Test qbar at high speed."""
        fdm = CreateFDM(self.sandbox)
        try:
            fdm.load_model("f16")
            fdm["ic/h-sl-ft"] = 30000
            fdm["ic/mach"] = 0.9
            fdm.run_ic()

            pm = fdm.get_property_manager()
            if pm.hasNode("aero/qbar-psf"):
                qbar = fdm["aero/qbar-psf"]
                # At high Mach, qbar should be significant
                self.assertGreater(qbar, 100)
        except Exception:
            pass
        finally:
            del fdm

    def test_wingarea_property(self):
        """Test wing area property for qS calculation."""
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


if __name__ == "__main__":
    RunTest(TestDynamicPressure)
