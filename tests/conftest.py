# conftest.py
#
# Pytest configuration and shared fixtures for JSBSim tests.
#
# Provides common fixtures for FDM setup, sandbox management, and simulation execution.
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
import sys

import pytest

# Make JSBSim module importable
sys.path.insert(0, os.path.dirname(__file__))

from JSBSim_utils import CreateFDM, SandBox

import jsbsim

# Silence debug messages from JSBSim
jsbsim.FGJSBBase().debug_lvl = 0


@pytest.fixture
def sandbox():
    """
    Fixture providing a temporary sandbox directory for test isolation.

    Automatically cleans up after test completion.

    Usage:
        def test_something(sandbox):
            script_path = sandbox.path_to_jsbsim_file('scripts', 'c1722.xml')
    """
    box = SandBox()
    yield box
    box.erase()


@pytest.fixture
def fdm(sandbox):
    """
    Fixture providing an initialized FDMExec instance.

    The FDM is created in a sandboxed environment and paths are configured.
    Automatically cleans up after test completion.

    Usage:
        def test_simulation(fdm):
            fdm['ic/h-sl-ft'] = 5000
            fdm.run_ic()
    """
    _fdm = CreateFDM(sandbox)
    yield _fdm
    # FDM cleanup handled by sandbox cleanup
    del _fdm


@pytest.fixture
def simulation_dir(sandbox):
    """
    Fixture providing the current simulation directory context.

    Changes to sandbox directory and restores original after test.
    Useful for tests that need to work with local files.

    Usage:
        def test_with_files(simulation_dir, fdm):
            # Current directory is now the sandbox
            # Files created here are isolated
    """
    original_dir = os.getcwd()
    os.chdir(sandbox())
    yield sandbox()
    os.chdir(original_dir)


@pytest.fixture(scope="session")
def aircraft_list(request):
    """
    Fixture providing list of available aircraft for parametrized tests.

    Used for testing against multiple aircraft configurations.
    Scope: session (computed once per test session)

    Usage:
        @pytest.mark.parametrize('aircraft', aircraft_list)
        def test_aircraft(aircraft, fdm):
            pass
    """
    jsbsim_path = os.path.join(os.path.dirname(__file__), "..", "aircraft")
    if os.path.exists(jsbsim_path):
        return [d for d in os.listdir(jsbsim_path) if os.path.isdir(os.path.join(jsbsim_path, d))]
    return []


@pytest.fixture(scope="session")
def script_list(request):
    """
    Fixture providing list of available scripts for parametrized tests.

    Used for testing script execution across available scripts.
    Scope: session (computed once per test session)

    Usage:
        @pytest.mark.parametrize('script', script_list)
        def test_script(script, fdm, sandbox):
            pass
    """
    jsbsim_path = os.path.join(os.path.dirname(__file__), "..", "scripts")
    if os.path.exists(jsbsim_path):
        return [f for f in os.listdir(jsbsim_path) if f.endswith(".xml")]
    return []


def pytest_configure(config):
    """Pytest hook for custom configuration at session start."""
    # Add custom configuration here if needed
    pass


def pytest_collection_modifyitems(config, items):
    """
    Pytest hook to automatically mark tests based on module location.

    Automatically applies markers based on test file location:
    - Tests in integration_tests/ are marked with @pytest.mark.integration
    - Tests in unit_tests/ are marked with @pytest.mark.unit
    """
    for item in items:
        # Mark integration tests
        if "integration_tests" in str(item.fspath):
            item.add_marker(pytest.mark.integration)
        # Mark unit tests
        elif "unit_tests" in str(item.fspath):
            item.add_marker(pytest.mark.unit)


def pytest_sessionstart(session):
    """Pytest hook called at test session start."""
    # Optional: Print test configuration info
    pass
