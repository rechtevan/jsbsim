# JSBSim Integration Tests

This directory contains integration tests for JSBSim that validate full simulation workflows, end-to-end behavior, and system interactions.

## Purpose

Integration tests verify that multiple components of JSBSim work correctly together. Unlike unit tests that test isolated components, integration tests:

- Execute complete simulation scenarios
- Validate script execution from start to finish
- Test interactions between multiple subsystems (propulsion, aerodynamics, FCS, etc.)
- Verify output file generation and format
- Test real-world use cases and workflows

## Test Suite Organization (10 Scenarios)

These tests are designed to increase code coverage from 24.9% to 40%+ by exercising realistic flight scenarios:

### Phase 1 - Framework (COMPLETED)
1. **test_01_aircraft_loading.py** - Aircraft loading and property initialization (+2-3% coverage)
   - Tests FGFDMExec, FGXMLElement, FGPropertyManager, FGAircraft
   - Verifies aircraft can be loaded and properties initialized
   - Foundation test for all others

### Phase 2 - Flight Scenarios (COMPLETED)
2. **test_02_takeoff_sequence.py** - Ground to flight transition (+3-4% coverage)
   - Tests takeoff roll, rotation, and initial climb
   - Validates ground-to-air transition dynamics

3. **test_03_landing_approach.py** - Descent and landing dynamics (+2-3% coverage)
   - Tests approach, flare, and touchdown
   - Validates landing gear and ground contact

4. **test_04_climb_descent.py** - Climb and descent scenarios (+2-3% coverage)
   - Tests altitude changes and energy management
   - Validates vertical navigation

5. **test_05_coordinated_turns.py** - Flight control system testing (+2-3% coverage)
   - Tests coordinated turns and bank angles
   - Validates lateral-directional dynamics

### Phase 3 - Script Execution (COMPLETED)
6. **test_06_script_execution.py** - Script parsing and execution (+3-4% coverage)
   - Tests script loading and completion
   - Validates simulation sequencing

7. **test_07_events_conditions.py** - Event system and conditions (+2-3% coverage)
   - Tests event triggering and condition evaluation
   - Validates scripted automation

### Phase 4 - Advanced Testing (COMPLETED)
8. **test_08_initialization_trimming.py** - Initialization and trimming (+4-5% coverage)
   - Unit tests: IC parameter setting and validation
   - Integration tests: Trim convergence and subsystem coordination
   - E2E tests: Complete trim workflows and linearization

9. **test_09_subsystem_interactions.py** - Subsystem interactions (+4-5% coverage)
   - Unit tests: Subsystem interface validation
   - Integration tests: Multi-subsystem coordination
   - E2E tests: Complete flights with all subsystems

10. **test_10_aircraft_specific_scenarios.py** - Aircraft-specific E2E (+3-4% coverage)
    - C172: General aviation scenarios
    - F-16: High-performance flight
    - Ball: Basic physics validation
    - 737: Commercial transport operations

**Expected total coverage gain: +15-20% (from 24.9% to 40%+)**

**Status**: Phase 4 tests implemented. Some tests require debugging for aircraft configuration and IC setup issues.

## Directory Structure

```
tests/integration_tests/
├── __init__.py                           # Package initialization
├── conftest.py                           # Integration-specific fixtures
├── README.md                             # This file
├── test_01_aircraft_loading.py          # Phase 1: Aircraft loading
├── test_02_takeoff_sequence.py          # Phase 2: Takeoff scenarios
├── test_03_landing_approach.py          # Phase 2: Landing scenarios
├── test_04_climb_descent.py             # Phase 2: Climb/descent
├── test_05_coordinated_turns.py         # Phase 2: Coordinated turns
├── test_06_script_execution.py          # Phase 3: Script execution
├── test_07_events_conditions.py         # Phase 3: Events and conditions
├── test_08_initialization_trimming.py   # Phase 4: Init and trimming
├── test_09_subsystem_interactions.py    # Phase 4: Subsystem interactions
└── test_10_aircraft_specific_scenarios.py  # Phase 4: Aircraft-specific tests
```

## Running Integration Tests

**Important**: These integration tests use the unittest framework and should be run directly as Python scripts, not via pytest.

### Run All Integration Tests

```bash
# From the jsbsim root directory
# Set library path and run each test file
export LD_LIBRARY_PATH=$PWD/build/src:$LD_LIBRARY_PATH

# Run all test files
for test in tests/integration_tests/test_*.py; do
    python "$test"
done
```

### Run Specific Test Files

```bash
# Set library path
export LD_LIBRARY_PATH=$PWD/build/src:$LD_LIBRARY_PATH

# Run specific test
python tests/integration_tests/test_08_initialization_trimming.py
```

### Alternative: Run via pytest (compatibility mode)

Note: Pytest compatibility may be limited. Unittest is recommended.

```bash
pytest tests/integration_tests/test_01_aircraft_loading.py -v
```

### Run Tests Matching a Pattern

```bash
# Run tests with "propulsion" in the name
pytest tests/integration_tests/ -k propulsion
```

### Exclude Slow Tests

```bash
# Skip tests marked as slow
pytest tests/integration_tests/ -m "not slow"
```

### Run Specific Markers

```bash
# Run only propulsion tests
pytest -m propulsion

# Run script execution tests
pytest -m script

# Run smoke tests (quick sanity checks)
pytest -m smoke
```

## Available Fixtures

Integration tests inherit all fixtures from the root `tests/conftest.py` plus additional integration-specific fixtures.

### Core Fixtures (from tests/conftest.py)

1. **sandbox**: Temporary isolated directory for test execution
   ```python
   def test_example(sandbox):
       script_path = sandbox.path_to_jsbsim_file('scripts', 'c1722.xml')
   ```

2. **fdm**: Initialized FGFDMExec instance in sandbox
   ```python
   def test_example(fdm):
       fdm['ic/h-sl-ft'] = 5000
       fdm.run_ic()
   ```

3. **simulation_dir**: Changes to sandbox directory context
   ```python
   def test_example(simulation_dir, fdm):
       # Current directory is now the sandbox
   ```

4. **aircraft_list**: List of available aircraft (session scope)
   ```python
   @pytest.mark.parametrize('aircraft', aircraft_list)
   def test_aircraft(aircraft, fdm):
       pass
   ```

5. **script_list**: List of available scripts (session scope)
   ```python
   @pytest.mark.parametrize('script', script_list)
   def test_script(script, fdm):
       pass
   ```

### Integration-Specific Fixtures (from tests/integration_tests/conftest.py)

1. **script_runner**: Execute scripts to completion
   ```python
   def test_script_execution(script_runner):
       fdm = script_runner('c1722.xml')
       assert fdm is not None
   ```

2. **sim_executor**: Run simulation for specified duration
   ```python
   def test_timed_simulation(sim_executor, fdm):
       sim_executor(10.0)  # Run for 10 seconds
   ```

3. **aircraft_loader**: Load aircraft definitions
   ```python
   def test_aircraft(aircraft_loader):
       fdm, tree, aircraft_name, path = aircraft_loader('c172/c1722.xml')
   ```

4. **get_property**: Cleaner property getter syntax
   ```python
   def test_properties(get_property, fdm):
       height = get_property('position/h-sl-ft')
   ```

5. **set_property**: Cleaner property setter syntax
   ```python
   def test_properties(set_property, fdm):
       set_property('ic/h-sl-ft', 5000)
   ```

6. **property_monitor**: Monitor property changes during simulation
   ```python
   def test_altitude_climb(property_monitor, fdm):
       with property_monitor('position/h-sl-ft') as monitor:
           fdm.run_ic()
           for _ in range(100):
               fdm.run()
       assert monitor.final > monitor.initial
   ```

7. **output_validator**: Validate simulation output files
   ```python
   def test_output(output_validator):
       assert output_validator.csv_exists('JSBout172B.csv')
       df = output_validator.read_csv('JSBout172B.csv')
       assert len(df) > 100
   ```

8. **fdm_state_snapshot**: Capture and compare FDM state
   ```python
   def test_state_changes(fdm_state_snapshot, fdm):
       state1 = fdm_state_snapshot()
       # ... run simulation ...
       state2 = fdm_state_snapshot()
       assert state1['position/h-sl-ft'] != state2['position/h-sl-ft']
   ```

## Test Markers

Use markers to organize and categorize tests:

```python
@pytest.mark.integration
def test_full_simulation():
    pass

@pytest.mark.slow
def test_long_running_simulation():
    pass

@pytest.mark.propulsion
def test_engine_startup():
    pass

@pytest.mark.script
def test_script_execution():
    pass

@pytest.mark.smoke
def test_basic_functionality():
    pass
```

Available markers:
- `integration`: Integration tests (automatically applied to this directory)
- `slow`: Tests that take significant time to run
- `propulsion`: Propulsion system tests
- `aerodynamics`: Aerodynamics system tests
- `control`: Flight control system tests
- `propag`: Propagation/dynamics tests
- `script`: Script execution tests
- `socket`: Socket I/O tests
- `regression`: Regression tests
- `smoke`: Quick sanity check tests

## Writing Integration Tests

### Basic Integration Test Template

```python
import pytest

def test_basic_simulation(fdm, sandbox):
    """Test basic simulation execution."""
    # Load aircraft and initialize
    fdm.load_model('c172x')
    fdm['ic/h-sl-ft'] = 5000
    fdm['ic/vc-kts'] = 100
    fdm.run_ic()

    # Run simulation
    for _ in range(100):
        assert fdm.run()

    # Verify results
    altitude = fdm['position/h-sl-ft']
    assert altitude > 0
```

### Script Execution Test

```python
def test_script_execution(script_runner):
    """Test script runs to completion."""
    fdm = script_runner('c1722.xml')
    assert fdm is not None
    assert fdm['simulation/sim-time-sec'] > 0
```

### Property Monitoring Test

```python
def test_climb_performance(property_monitor, fdm, sandbox):
    """Test aircraft climb performance."""
    fdm.load_model('c172x')
    fdm['ic/h-sl-ft'] = 1000
    fdm.run_ic()

    with property_monitor('position/h-sl-ft') as monitor:
        for _ in range(1000):
            fdm['fcs/elevator-cmd-norm'] = -0.1
            fdm.run()
            monitor.update()

    # Verify aircraft climbed
    assert monitor.final > monitor.initial
    assert monitor.max > monitor.min
```

### Output Validation Test

```python
def test_output_generation(script_runner, output_validator, simulation_dir):
    """Test simulation output file generation."""
    fdm = script_runner('c1722.xml')

    # Verify output file exists
    assert output_validator.csv_exists('JSBout172B.csv')

    # Validate output data
    assert output_validator.verify_monotonic_time('JSBout172B.csv')
    assert output_validator.verify_no_nans('JSBout172B.csv')
```

### Parametrized Test

```python
@pytest.mark.parametrize('script', ['c1722.xml', 'ball.xml', 'Short_S23_3.xml'])
def test_multiple_scripts(script, script_runner):
    """Test multiple scripts execute successfully."""
    fdm = script_runner(script)
    assert fdm is not None
```

## Best Practices

1. **Isolation**: Each test should be independent and not rely on other tests
2. **Cleanup**: Use fixtures for automatic cleanup (sandbox, fdm)
3. **Markers**: Tag tests appropriately for easy selection
4. **Documentation**: Include docstrings explaining what each test validates
5. **Assertions**: Use clear, specific assertions with meaningful messages
6. **Naming**: Use descriptive test names: `test_<what>_<condition>_<expected>`
7. **Speed**: Mark slow tests with `@pytest.mark.slow` so they can be skipped
8. **Data**: Use sandbox for file I/O to avoid polluting the repository

## Continuous Integration

Integration tests should be run:
- Before committing significant changes
- In CI/CD pipelines
- Before releases

To run a quick smoke test suite:
```bash
pytest -m smoke tests/integration_tests/
```

## Troubleshooting

### Tests Fail with Library Not Found

If you see `ImportError: libJSBSim.so.1: cannot open shared object file`:

```bash
# From build directory
export LD_LIBRARY_PATH=$PWD/src:$LD_LIBRARY_PATH
pytest tests/integration_tests/
```

### Tests Fail to Find Aircraft/Scripts

Ensure you're running tests from the JSBSim root directory, or that the sandbox fixture is being used correctly.

### Slow Test Execution

Use markers to skip slow tests during development:
```bash
pytest -m "integration and not slow"
```

## Scenario 1: Aircraft Loading (IMPLEMENTED)

### Overview

File: `test_01_aircraft_loading.py`

The aircraft loading test provides the foundation for all other integration tests by verifying that JSBSim can successfully load various aircraft types and correctly initialize all critical properties.

### What It Tests

1. **Aircraft Loading**: Multiple aircraft types (C172P, F-16, ball, C172X)
2. **Property System**: All critical properties correctly initialized
   - Position (altitude, latitude, longitude)
   - Velocity (airspeed, body velocities u/v/w)
   - Attitude (roll, pitch, yaw)
   - Flight controls (throttle, elevator, aileron, rudder)
3. **Mass Properties**: Weight, CG location, moments of inertia
4. **Engine Configuration**: Piston and turbine engine properties
5. **Aerodynamic References**: Wing area, span, chord
6. **Initial Conditions**: IC system works correctly
7. **XML Parsing**: Aircraft XML files parsed without errors

### Components Exercised

- `FGFDMExec` - Main executive initialization
- `FGXMLElement` - XML parsing and validation
- `FGPropertyManager` - Property tree registration
- `FGAircraft` - Aircraft configuration assembly
- `FGInitialCondition` - Initial condition system
- `FGMassBalance` - Mass properties initialization
- `FGPropulsion` - Engine configuration loading
- `FGAerodynamics` - Aerodynamic reference values

### Test Methods

| Test Method | Description |
|-------------|-------------|
| `test_load_c172p()` | Load Cessna 172P and verify all properties |
| `test_load_f16()` | Load F-16 fighter and verify properties |
| `test_load_ball()` | Load simple ball model |
| `test_load_c172x()` | Load C172X variant |
| `test_multiple_aircraft_loading()` | Load multiple aircraft sequentially |
| `test_property_tree_navigation()` | Verify property tree structure |
| `test_mass_properties_loaded()` | Verify mass, CG, inertia |
| `test_engine_configuration_loaded()` | Verify engine properties |
| `test_aerodynamic_reference_values()` | Verify wing area, span, chord |
| `test_initial_conditions_system()` | Verify IC system functionality |
| `test_xml_parsing_validation()` | Verify XML parsing correctness |

### Running Scenario 1

**Prerequisites**: JSBSim must be built with Python support:

```bash
cd build
cmake -DBUILD_PYTHON_MODULE=ON ..
make -j$(nproc)
```

**Run all tests in Scenario 1**:
```bash
python -m pytest ../tests/integration_tests/test_01_aircraft_loading.py -v
```

**Run specific test**:
```bash
python -m pytest ../tests/integration_tests/test_01_aircraft_loading.py::TestAircraftLoading::test_load_c172p -v
```

**Run as standalone**:
```bash
python ../tests/integration_tests/test_01_aircraft_loading.py
```

### Expected Output

```
test_01_aircraft_loading.py::TestAircraftLoading::test_load_c172p PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_load_f16 PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_load_ball PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_load_c172x PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_multiple_aircraft_loading PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_property_tree_navigation PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_mass_properties_loaded PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_engine_configuration_loaded PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_aerodynamic_reference_values PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_initial_conditions_system PASSED
test_01_aircraft_loading.py::TestAircraftLoading::test_xml_parsing_validation PASSED

============================== 11 passed in X.XX seconds ==============================
```

### Expected Coverage Impact

**+2-3% line coverage** in these files:
- `src/FGFDMExec.cpp` - Aircraft loading, executive initialization
- `src/input_output/FGXMLElement.cpp` - XML parsing
- `src/models/FGAircraft.cpp` - Aircraft assembly
- `src/models/FGMassBalance.cpp` - Mass property initialization
- `src/models/FGPropulsion.cpp` - Engine configuration
- `src/initialization/FGInitialCondition.cpp` - IC system

### Coverage Analysis

After running this test, generate a coverage report:

```bash
cd build-coverage
cmake -DENABLE_COVERAGE=ON -DBUILD_PYTHON_MODULE=ON ..
make -j$(nproc)
python -m pytest ../tests/integration_tests/test_01_aircraft_loading.py
make lcov
xdg-open lcov/html/all_targets/index.html
```

## Further Information

- See `tests/conftest.py` for core fixture implementations
- See `tests/integration_tests/conftest.py` for integration fixture implementations
- See `pytest.ini` for pytest configuration
- See existing tests in `tests/` for examples
- JSBSim documentation: https://jsbsim-team.github.io/jsbsim/
- Integration test planning: `.local/INTEGRATION-TEST-QUICK-START.md`
- Detailed scenarios: `.local/integration-test-scenarios.md`
