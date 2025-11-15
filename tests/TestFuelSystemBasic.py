# TestFuelSystemBasic.py
#
# Basic regression tests for fuel system functionality.
# Tests fuel tanks, fuel flow, fuel depletion, and multiple tank configurations.
#
# Copyright (c) 2025 rechtevan
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


class TestFuelSystemBasic(JSBSimTestCase):
    """
    Basic tests for fuel system functionality.

    These tests verify:
    - Fuel tank property access (capacity, contents, position)
    - Fuel depletion during engine operation
    - Fuel flow rate calculations
    - Multiple fuel tank configurations
    - Fuel density properties
    - Total fuel calculations
    - Fuel exhaustion behavior

    Uses c172x aircraft which has:
    - 2 fuel tanks (left and right wing tanks)
    - Each tank: 130 lbs capacity
    - Total capacity: 260 lbs (~43.3 gallons at 6 lbs/gal)
    - Single piston engine fed from both tanks
    """

    def test_fuel_tank_properties(self):
        """Test basic fuel tank property access for capacity, contents, and position."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test tank 0 properties (capacity calculated from 100% full contents)
        contents_0 = fdm["propulsion/tank[0]/contents-lbs"]
        pct_full_0 = fdm["propulsion/tank[0]/pct-full"]

        self.assertIsNotNone(contents_0, "Tank 0 contents should be accessible")
        self.assertGreaterEqual(contents_0, 0.0, "Tank 0 contents should be non-negative")
        self.assertAlmostEqual(pct_full_0, 100.0, delta=1.0, msg="Tank 0 should start at 100% full")

        # Calculate capacity from 100% full tank
        capacity_lbs_0 = contents_0  # Since tank starts at 100%
        self.assertEqual(capacity_lbs_0, 130.0, "Tank 0 capacity should be 130 lbs per c172x.xml")

        # Test tank 1 properties
        contents_1 = fdm["propulsion/tank[1]/contents-lbs"]
        pct_full_1 = fdm["propulsion/tank[1]/pct-full"]

        self.assertIsNotNone(contents_1, "Tank 1 contents should be accessible")
        self.assertGreaterEqual(contents_1, 0.0, "Tank 1 contents should be non-negative")
        self.assertAlmostEqual(pct_full_1, 100.0, delta=1.0, msg="Tank 1 should start at 100% full")

        # Verify both tanks have same capacity (symmetrical wing tanks)
        capacity_lbs_1 = contents_1
        self.assertEqual(
            capacity_lbs_0,
            capacity_lbs_1,
            "Wing tanks should have equal capacity for c172x",
        )

        del fdm

    def test_fuel_depletion_during_flight(self):
        """Test that fuel decreases over time during engine operation."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.load_ic("reset01", True)  # Load IC with engine running
        fdm.run_ic()

        # Set throttle and mixture for steady cruise
        fdm["fcs/throttle-cmd-norm"] = 0.7
        fdm["fcs/mixture-cmd-norm"] = 0.85

        # Record initial fuel levels
        initial_fuel_0 = fdm["propulsion/tank[0]/contents-lbs"]
        initial_fuel_1 = fdm["propulsion/tank[1]/contents-lbs"]
        initial_total = fdm["propulsion/total-fuel-lbs"]

        self.assertGreater(initial_total, 0.0, "Should have fuel at start of simulation")

        # Run for 60 seconds (600 iterations at 0.1 sec/iteration)
        for _ in range(600):
            fdm.run()

        # Record final fuel levels
        final_fuel_0 = fdm["propulsion/tank[0]/contents-lbs"]
        final_fuel_1 = fdm["propulsion/tank[1]/contents-lbs"]
        final_total = fdm["propulsion/total-fuel-lbs"]

        # Verify fuel was consumed from both tanks
        fuel_burned_0 = initial_fuel_0 - final_fuel_0
        fuel_burned_1 = initial_fuel_1 - final_fuel_1
        total_burned = initial_total - final_total

        self.assertGreater(fuel_burned_0, 0.0, "Tank 0 should have consumed fuel after 60 seconds")
        self.assertGreater(fuel_burned_1, 0.0, "Tank 1 should have consumed fuel after 60 seconds")
        self.assertGreater(total_burned, 0.0, "Total fuel should have decreased after 60 seconds")

        # Verify fuel consumption is reasonable (not too high or too low)
        # C172 at idle/low power should burn around 0.05-0.5 lbs in 60 seconds
        self.assertGreater(total_burned, 0.01, "Should burn more than 0.01 lbs in 60 seconds")
        self.assertLess(total_burned, 2.0, "Should burn less than 2 lbs in 60 seconds")

        del fdm

    def test_fuel_flow_rate(self):
        """Test that fuel flow rate property matches actual fuel consumption."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.load_ic("reset01", True)
        fdm.run_ic()

        # Set throttle and mixture
        fdm["fcs/throttle-cmd-norm"] = 0.6
        fdm["fcs/mixture-cmd-norm"] = 0.9

        # Run a few iterations to stabilize
        for _ in range(50):
            fdm.run()

        # Record initial fuel
        initial_total = fdm["propulsion/total-fuel-lbs"]

        # Run for 10 seconds and average fuel flow rate
        dt = fdm.get_delta_t()
        num_iterations = int(10.0 / dt)

        total_fuel_flow = 0.0
        for _ in range(num_iterations):
            fdm.run()
            total_fuel_flow += fdm["propulsion/engine[0]/fuel-flow-rate-pps"] * dt

        final_total = fdm["propulsion/total-fuel-lbs"]
        actual_burned = initial_total - final_total

        self.assertGreater(actual_burned, 0.0, "Fuel should be consumed during flight")
        self.assertGreater(total_fuel_flow, 0.0, "Fuel flow should be positive")

        # Verify actual consumption is close to integrated fuel flow
        # Allow 50% tolerance since fuel flow varies with conditions
        self.assertAlmostEqual(
            actual_burned,
            total_fuel_flow,
            delta=max(actual_burned, total_fuel_flow) * 0.5,
            msg=f"Actual fuel burned ({actual_burned:.3f} lbs) should match "
            f"integrated fuel flow ({total_fuel_flow:.3f} lbs)",
        )

        del fdm

    def test_multiple_fuel_tanks(self):
        """Test aircraft with multiple fuel tanks and verify independent properties."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # c172x has 2 fuel tanks (known from aircraft definition)
        num_tanks = 2

        # Test that we can access properties for all tanks
        for i in range(num_tanks):
            contents = fdm[f"propulsion/tank[{i}]/contents-lbs"]
            pct_full = fdm[f"propulsion/tank[{i}]/pct-full"]

            self.assertIsNotNone(contents, f"Tank {i} contents should be accessible")
            self.assertIsNotNone(pct_full, f"Tank {i} percent full should be accessible")

            self.assertGreaterEqual(pct_full, 0.0, f"Tank {i} percent full should be >= 0")
            self.assertLessEqual(pct_full, 100.0, f"Tank {i} percent full should be <= 100")

        # Test modifying individual tank contents
        original_0 = fdm["propulsion/tank[0]/contents-lbs"]
        original_1 = fdm["propulsion/tank[1]/contents-lbs"]

        # Reduce tank 0 by 50%
        fdm["propulsion/tank[0]/contents-lbs"] = original_0 * 0.5

        # Verify tank 0 changed but tank 1 did not
        new_0 = fdm["propulsion/tank[0]/contents-lbs"]
        new_1 = fdm["propulsion/tank[1]/contents-lbs"]

        self.assertAlmostEqual(
            new_0,
            original_0 * 0.5,
            delta=0.1,
            msg="Tank 0 should be at 50% of original",
        )
        self.assertAlmostEqual(new_1, original_1, delta=0.01, msg="Tank 1 should be unchanged")

        del fdm

    def test_fuel_tank_selection(self):
        """Test fuel tank feed selection and engine feed configuration."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.load_ic("reset01", True)
        fdm.run_ic()

        # c172x engine can feed from both tanks (feed 0 and feed 1)
        # Verify the engine is connected to both tanks by checking fuel consumption

        # Set throttle and mixture
        fdm["fcs/throttle-cmd-norm"] = 0.5
        fdm["fcs/mixture-cmd-norm"] = 0.85

        initial_0 = fdm["propulsion/tank[0]/contents-lbs"]
        initial_1 = fdm["propulsion/tank[1]/contents-lbs"]

        # Run for 30 seconds
        for _ in range(300):
            fdm.run()

        final_0 = fdm["propulsion/tank[0]/contents-lbs"]
        final_1 = fdm["propulsion/tank[1]/contents-lbs"]

        # Both tanks should have consumed fuel (engine feeds from both)
        burned_0 = initial_0 - final_0
        burned_1 = initial_1 - final_1

        self.assertGreater(burned_0, 0.0, "Tank 0 should supply fuel to engine")
        self.assertGreater(burned_1, 0.0, "Tank 1 should supply fuel to engine")

        # For symmetric tanks with equal feed, consumption should be roughly equal
        # Allow 20% difference to account for any feed priority or timing differences
        consumption_ratio = burned_0 / burned_1 if burned_1 > 0 else 0.0
        self.assertGreater(
            consumption_ratio,
            0.8,
            "Tank consumption should be roughly balanced",
        )
        self.assertLess(
            consumption_ratio,
            1.2,
            "Tank consumption should be roughly balanced",
        )

        del fdm

    def test_fuel_density(self):
        """Test fuel density property and volume/weight conversions."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Test fuel density property (standard AvGas density ~6 lbs/gal)
        density_0 = fdm["propulsion/tank[0]/density-lbs_per_gal"]
        density_1 = fdm["propulsion/tank[1]/density-lbs_per_gal"]

        self.assertIsNotNone(density_0, "Tank 0 density should be accessible")
        self.assertIsNotNone(density_1, "Tank 1 density should be accessible")

        self.assertGreater(density_0, 5.0, "Fuel density should be > 5 lbs/gal")
        self.assertLess(density_0, 7.0, "Fuel density should be < 7 lbs/gal")

        # Both tanks should have same fuel type/density
        self.assertAlmostEqual(
            density_0,
            density_1,
            delta=0.01,
            msg="Both tanks should have same fuel density",
        )

        # Test volume calculation: volume = weight / density
        contents_lbs_0 = fdm["propulsion/tank[0]/contents-lbs"]
        contents_gal_0 = fdm["propulsion/tank[0]/contents-volume-gal"]

        # Calculated volume should match reported volume
        calculated_volume = contents_lbs_0 / density_0

        self.assertAlmostEqual(
            contents_gal_0,
            calculated_volume,
            delta=0.1,
            msg="Volume calculation should match weight/density formula",
        )

        # Test that changing fuel amount updates volume proportionally
        new_contents_lbs = contents_lbs_0 * 0.6
        fdm["propulsion/tank[0]/contents-lbs"] = new_contents_lbs

        # Run one iteration to allow FCS to recalculate volume
        fdm.run()

        new_contents_gal = fdm["propulsion/tank[0]/contents-volume-gal"]
        expected_volume = new_contents_lbs / density_0

        self.assertAlmostEqual(
            new_contents_gal,
            expected_volume,
            delta=0.1,
            msg="Volume should update when fuel weight changes",
        )

        del fdm

    def test_total_fuel_calculation(self):
        """Test that total fuel correctly sums all tank contents."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.run_ic()

        # Get individual tank contents
        tank_0 = fdm["propulsion/tank[0]/contents-lbs"]
        tank_1 = fdm["propulsion/tank[1]/contents-lbs"]

        # Get total fuel
        total_fuel = fdm["propulsion/total-fuel-lbs"]

        # Verify total equals sum of tanks
        expected_total = tank_0 + tank_1

        self.assertAlmostEqual(
            total_fuel,
            expected_total,
            delta=0.01,
            msg="Total fuel should equal sum of all tank contents",
        )

        # Modify one tank and verify total updates
        fdm["propulsion/tank[0]/contents-lbs"] = tank_0 * 0.7
        fdm.run()  # Run one iteration to update totals

        new_tank_0 = fdm["propulsion/tank[0]/contents-lbs"]
        new_total = fdm["propulsion/total-fuel-lbs"]
        new_expected_total = new_tank_0 + tank_1

        self.assertAlmostEqual(
            new_total,
            new_expected_total,
            delta=0.01,
            msg="Total fuel should update when tank contents change",
        )

        # Modify both tanks
        fdm["propulsion/tank[0]/contents-lbs"] = 50.0
        fdm["propulsion/tank[1]/contents-lbs"] = 75.0
        fdm.run()  # Run one iteration to update totals

        final_total = fdm["propulsion/total-fuel-lbs"]
        self.assertAlmostEqual(
            final_total,
            125.0,
            delta=0.01,
            msg="Total fuel should be 50 + 75 = 125 lbs",
        )

        del fdm

    def test_fuel_exhaustion(self):
        """Test behavior when fuel runs out during flight."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.load_ic("reset01", True)
        fdm.run_ic()

        # Set minimal fuel in both tanks (5 lbs each = ~10 lbs total)
        fdm["propulsion/tank[0]/contents-lbs"] = 5.0
        fdm["propulsion/tank[1]/contents-lbs"] = 5.0

        # Set engine at high throttle to burn fuel quickly
        fdm["fcs/throttle-cmd-norm"] = 1.0
        fdm["fcs/mixture-cmd-norm"] = 0.95

        # Run until engine stops or max iterations
        max_iterations = 20000  # Prevent infinite loop (more iterations for low fuel)
        iterations = 0

        while iterations < max_iterations:
            fdm.run()
            iterations += 1

            # Check if engine has stopped
            engine_running = fdm["propulsion/engine[0]/set-running"]
            if engine_running == 0:
                break

            # Check if fuel is exhausted
            total_fuel = fdm["propulsion/total-fuel-lbs"]
            if total_fuel <= 0.1:  # Nearly empty
                # Give a few more iterations for engine to notice
                for _ in range(10):
                    fdm.run()
                break

        # Verify fuel was significantly depleted
        final_total_fuel = fdm["propulsion/total-fuel-lbs"]
        initial_fuel = 10.0  # Started with 10 lbs total
        fuel_consumed = initial_fuel - final_total_fuel

        # Should have consumed at least 30% of initial fuel
        self.assertGreater(
            fuel_consumed,
            initial_fuel * 0.3,
            f"Should consume at least 30% of fuel ({initial_fuel * 0.3:.1f} lbs), "
            f"but only consumed {fuel_consumed:.2f} lbs",
        )

        # If engine stopped, verify fuel is very low
        final_engine_state = fdm["propulsion/engine[0]/set-running"]
        if final_engine_state == 0:
            # Engine stopped - verify fuel is very low
            self.assertLess(
                final_total_fuel,
                3.0,
                "Engine stopped, so fuel should be nearly empty",
            )

            # Verify fuel flow drops to zero
            final_fuel_flow = fdm["propulsion/engine[0]/fuel-flow-rate-pps"]
            self.assertAlmostEqual(
                final_fuel_flow,
                0.0,
                delta=0.01,
                msg="Fuel flow should be zero when engine stops",
            )

        del fdm

    def test_fuel_percent_full(self):
        """Test that percent-full calculation is accurate."""
        fdm = CreateFDM(self.sandbox)
        fdm.load_model("c172x")
        fdm.load_ic("reset01", True)
        fdm.run_ic()

        # Get initial percent full for both tanks
        pct_0 = fdm["propulsion/tank[0]/pct-full"]
        pct_1 = fdm["propulsion/tank[1]/pct-full"]

        # Tanks start at 100% (130 lbs contents, 130 lbs capacity)
        self.assertAlmostEqual(pct_0, 100.0, delta=0.5, msg="Tank 0 should start at 100% full")
        self.assertAlmostEqual(pct_1, 100.0, delta=0.5, msg="Tank 1 should start at 100% full")

        # Set tank 0 to 50% of capacity
        capacity = 130.0  # lbs (from c172x.xml)
        fdm["propulsion/tank[0]/contents-lbs"] = capacity * 0.5

        new_pct_0 = fdm["propulsion/tank[0]/pct-full"]
        self.assertAlmostEqual(
            new_pct_0,
            50.0,
            delta=1.0,
            msg="Tank 0 should be 50% full after setting to half capacity",
        )

        # Set tank 1 to 25% of capacity
        fdm["propulsion/tank[1]/contents-lbs"] = capacity * 0.25

        new_pct_1 = fdm["propulsion/tank[1]/pct-full"]
        self.assertAlmostEqual(
            new_pct_1,
            25.0,
            delta=1.0,
            msg="Tank 1 should be 25% full after setting to quarter capacity",
        )

        # Set tank to empty (0%)
        fdm["propulsion/tank[0]/contents-lbs"] = 0.0

        empty_pct = fdm["propulsion/tank[0]/pct-full"]
        self.assertAlmostEqual(empty_pct, 0.0, delta=0.1, msg="Empty tank should show 0% full")

        del fdm

    def test_fuel_consumption_vs_throttle(self):
        """Test that fuel consumption increases with throttle setting."""
        # Test low throttle first
        fdm_low = CreateFDM(self.sandbox)
        fdm_low.load_model("c172x")
        fdm_low.load_ic("reset01", True)
        fdm_low.run_ic()

        fdm_low["fcs/throttle-cmd-norm"] = 0.3
        fdm_low["fcs/mixture-cmd-norm"] = 0.85

        # Stabilize and average fuel flow
        for _ in range(50):
            fdm_low.run()

        low_flow_samples = []
        for _ in range(100):
            fdm_low.run()
            low_flow_samples.append(fdm_low["propulsion/engine[0]/fuel-flow-rate-pps"])

        low_throttle_flow = sum(low_flow_samples) / len(low_flow_samples)
        del fdm_low

        # Test high throttle in separate FDM instance
        fdm_high = CreateFDM(self.sandbox)
        fdm_high.load_model("c172x")
        fdm_high.load_ic("reset01", True)
        fdm_high.run_ic()

        fdm_high["fcs/throttle-cmd-norm"] = 0.9
        fdm_high["fcs/mixture-cmd-norm"] = 0.85

        # Stabilize and average fuel flow
        for _ in range(50):
            fdm_high.run()

        high_flow_samples = []
        for _ in range(100):
            fdm_high.run()
            high_flow_samples.append(fdm_high["propulsion/engine[0]/fuel-flow-rate-pps"])

        high_throttle_flow = sum(high_flow_samples) / len(high_flow_samples)
        del fdm_high

        # High throttle should consume more fuel
        self.assertGreater(
            high_throttle_flow,
            low_throttle_flow,
            f"Higher throttle (90%) flow ({high_throttle_flow:.4f}) should exceed "
            f"low throttle (30%) flow ({low_throttle_flow:.4f})",
        )

        del self


RunTest(TestFuelSystemBasic)
