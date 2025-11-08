# conftest.py
#
# Pytest configuration specific to integration tests.
#
# Extends root conftest.py with integration-specific fixtures and settings.
#
# Copyright (c) 2025 Booz Allen Hamilton Inc.
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

import os

import pytest
from JSBSim_utils import CopyAircraftDef, ExecuteUntil, append_xml


@pytest.fixture
def script_runner(fdm, sandbox):
    """
    Fixture providing a utility function for running scripts to completion.

    Returns a callable that loads and executes a script to its end time.

    Usage:
        def test_script_execution(script_runner):
            result = script_runner('c1722.xml')
            assert result is not None
    """

    def run_script(script_name):
        """Load and run script to completion."""
        script_path = sandbox.path_to_jsbsim_file("scripts", script_name)
        if not os.path.isfile(script_path):
            script_path = sandbox.path_to_jsbsim_file("scripts", append_xml(script_name))

        fdm.load_script(script_path)
        fdm.run_ic()

        while fdm.run():
            pass

        return fdm

    return run_script


@pytest.fixture
def sim_executor(fdm):
    """
    Fixture providing a utility function for running simulation for a duration.

    Returns a callable that executes FDM until specified sim time.

    Usage:
        def test_timed_simulation(sim_executor):
            sim_executor(10.0)  # Run for 10 seconds of sim time
    """

    def execute_until(end_time):
        """Run simulation until specified sim time."""
        ExecuteUntil(fdm, end_time)
        return fdm

    return execute_until


@pytest.fixture
def aircraft_loader(fdm, sandbox):
    """
    Fixture providing utility to copy and load aircraft definitions.

    Returns a callable that handles aircraft definition file copying and loading.

    Usage:
        def test_aircraft(aircraft_loader):
            fdm = aircraft_loader('c172', 'c1722.xml')
    """

    def load_aircraft(script_path):
        """Copy aircraft definition and return FDM."""
        tree, aircraft_name, path = CopyAircraftDef(script_path, sandbox)
        return fdm, tree, aircraft_name, path

    return load_aircraft


@pytest.fixture
def get_property(fdm):
    """
    Fixture providing property getter with cleaner syntax.

    Returns a callable: prop_value = get_property('property/path')

    Usage:
        def test_properties(get_property, fdm):
            fdm.run_ic()
            height = get_property('position/h-sl-ft')
            assert height > 0
    """

    def get_prop(prop_name):
        """Get property value from FDM."""
        return fdm[prop_name]

    return get_prop


@pytest.fixture
def set_property(fdm):
    """
    Fixture providing property setter with cleaner syntax.

    Returns a callable: set_property('property/path', value)

    Usage:
        def test_properties(set_property, fdm):
            set_property('ic/h-sl-ft', 5000)
            set_property('ic/u-fps', 100)
            fdm.run_ic()
    """

    def set_prop(prop_name, value):
        """Set property value in FDM."""
        fdm[prop_name] = value

    return set_prop


@pytest.fixture
def property_monitor(fdm):
    """
    Fixture providing a context manager to monitor property changes.

    Useful for verifying that properties change during simulation.

    Usage:
        def test_altitude_climb(property_monitor, fdm):
            with property_monitor('position/h-sl-ft') as monitor:
                fdm.run_ic()
                for _ in range(100):
                    fdm['fcs/elevator-cmd-norm'] = 0.1
                    fdm.run()

            # monitor.initial, monitor.final, monitor.min, monitor.max available
            assert monitor.final > monitor.initial
    """

    class PropertyMonitor:
        def __init__(self, prop_name):
            self.prop_name = prop_name
            self.initial = None
            self.final = None
            self.min = None
            self.max = None
            self.values = []

        def __enter__(self):
            self.initial = fdm[self.prop_name]
            self.min = self.initial
            self.max = self.initial
            self.values = [self.initial]
            return self

        def __exit__(self, *args):
            self.final = fdm[self.prop_name]
            self.values.append(self.final)
            self.min = min(self.values)
            self.max = max(self.values)

        def update(self):
            """Call during simulation to record property value."""
            value = fdm[self.prop_name]
            self.values.append(value)
            self.min = min(self.min, value)
            self.max = max(self.max, value)

    def monitor(prop_name):
        return PropertyMonitor(prop_name)

    return monitor


@pytest.fixture
def output_validator():
    """
    Fixture providing utilities for validating simulation output files.

    Returns an object with methods for output validation.

    Usage:
        def test_output(output_validator):
            csv_file = 'JSBout172B.csv'
            assert output_validator.csv_exists(csv_file)
            df = output_validator.read_csv(csv_file)
            assert len(df) > 100
    """

    class OutputValidator:
        @staticmethod
        def csv_exists(filename):
            """Check if CSV output file exists."""
            return os.path.isfile(filename)

        @staticmethod
        def read_csv(filename):
            """Read CSV output file using pandas."""
            import pandas as pd

            return pd.read_csv(filename, index_col=0)

        @staticmethod
        def get_column(filename, column_name):
            """Get specific column from CSV output."""
            import pandas as pd

            df = pd.read_csv(filename, index_col=0)
            return df[column_name] if column_name in df.columns else None

        @staticmethod
        def verify_monotonic_time(filename):
            """Verify time column is monotonically increasing."""
            import pandas as pd

            df = pd.read_csv(filename, index_col=0)
            time_values = df.index.values
            return all(time_values[i] <= time_values[i + 1] for i in range(len(time_values) - 1))

        @staticmethod
        def verify_no_nans(filename):
            """Verify no NaN values in output."""
            import pandas as pd

            df = pd.read_csv(filename, index_col=0)
            return not df.isnull().any().any()

        @staticmethod
        def compare_with_reference(output_file, reference_file, tolerance=1e-3):
            """Compare output with reference file."""
            import numpy as np
            import pandas as pd

            output = pd.read_csv(output_file, index_col=0)
            reference = pd.read_csv(reference_file, index_col=0)

            # Check if they have same shape
            if output.shape != reference.shape:
                return False

            # Compare values with tolerance
            diff = np.abs(output - reference)
            return (diff <= tolerance).all().all()

    return OutputValidator()


@pytest.fixture
def fdm_state_snapshot(fdm):
    """
    Fixture providing utilities to capture and compare FDM state.

    Useful for regression testing and state verification.

    Usage:
        def test_state_changes(fdm_state_snapshot):
            snapshot1 = fdm_state_snapshot()
            # ... run simulation ...
            snapshot2 = fdm_state_snapshot()
            assert snapshot1.altitude != snapshot2.altitude
    """

    def capture_state(properties=None):
        """Capture current FDM state."""
        if properties is None:
            properties = [
                "position/h-sl-ft",
                "velocities/v-north-fps",
                "velocities/v-east-fps",
                "velocities/v-down-fps",
                "attitude/theta-rad",
                "attitude/phi-rad",
                "attitude/psi-true-rad",
            ]

        state = {}
        for prop in properties:
            try:
                state[prop] = fdm[prop]
            except (KeyError, Exception):
                state[prop] = None

        return state

    return capture_state


@pytest.fixture
def start_piston_engine(fdm):
    """
    Fixture providing a helper to properly configure piston engine for start.

    IMPORTANT: Call this BEFORE fdm.run_ic() to properly initialize the engine.

    Piston engines require specific initialization:
    - Mixture must be set to full rich (1.0)
    - Throttle should be set (0.5-1.0 for takeoff, 0.6-0.8 for cruise)
    - set-running property must be enabled

    Usage:
        def test_engine(start_piston_engine, fdm):
            fdm.load_model('c172x')
            fdm['ic/h-sl-ft'] = 0.0
            start_piston_engine(throttle=1.0)  # Set for takeoff
            fdm.run_ic()  # Now engine will be running
            # Engine should now be running with thrust
    """

    def configure_engine(engine_index=0, throttle=0.7, mixture=0.87):
        """
        Configure piston engine for start.

        Must be called BEFORE run_ic() for engine to start properly.
        After calling this and run_ic(), run simulation for ~30 frames
        to allow engine to fully start.

        Args:
            engine_index: Engine number (0 for single-engine aircraft)
            throttle: Throttle setting 0.0-1.0 (default 0.7 for good power)
            mixture: Mixture setting 0.0-1.0 (default 0.87 per scripts)

        Returns:
            None (call run_ic() then run ~30 frames after this)
        """
        # Set mixture (scripts use 0.87, not 1.0)
        fdm["fcs/mixture-cmd-norm"] = mixture

        # Set throttle
        fdm["fcs/throttle-cmd-norm"] = throttle

        # Turn on magnetos (both)
        fdm["propulsion/magneto_cmd"] = 3

        # Engage starter
        fdm["propulsion/starter_cmd"] = 1

        # Note: After this, caller must:
        # 1. Call fdm.run_ic()
        # 2. Run simulation for ~30 frames: for _ in range(30): fdm.run()
        # 3. Then engine should be running with thrust

    return configure_engine


# Hooks for integration test session setup/teardown


def pytest_configure(config):
    """Configure pytest for integration tests."""
    # Add integration test specific configuration
    pass


def pytest_sessionstart(session):
    """Run at beginning of integration test session."""
    # Optional: setup shared resources
    pass


def pytest_sessionfinish(session, exitstatus):
    """Run at end of integration test session."""
    # Optional: cleanup shared resources
    pass
