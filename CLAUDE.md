# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Context

**Fork Owner**: rechtevan (not upstream JSBSim-Team)
**Purpose**: Code examination, bug fixes, minor enhancements, security improvements, and test code development
**GitHub Issues**: Create issues in rechtevan's repository, not upstream

## Local Development Conventions

**`.local/` Directory**: Used for AI-generated analysis, scripts, reports, and other files that should NOT be committed to git. This directory is in `.gitignore`.

Use `.local/` for:
- Code analysis reports
- Test coverage reports
- Security scan results
- Build logs for review
- Temporary scripts and diagrams
- Any AI-generated content not part of the codebase

## Code Quality and Linting

**Pre-commit Hooks**: This repository uses pre-commit hooks for automated code quality checks. Hooks run automatically on `git commit`.

### Configured Linters and Formatters

**Python:**
- `black` - Code formatting (automatic fixes)
- `ruff` - Fast linting with auto-fixes
- `flake8` - Style checking
- `isort` - Import sorting
- `mypy` - Type checking (when configured)

**Markdown:**
- `markdownlint` - Markdown linting with auto-fixes
  - Disabled rules: MD013 (line length), MD033 (inline HTML), MD041 (first line h1)

**General:**
- Trailing whitespace trimming
- End-of-file newline enforcement
- Mixed line ending fixes
- Large file detection
- Merge conflict marker detection
- Private key detection (security)

**Language-Specific (when files present):**
- CMake formatting and linting
- XML validation (JSBSim config files)
- YAML/JSON/TOML syntax checking

### Pre-commit Commands

```bash
# Install pre-commit hooks
pre-commit install

# Run hooks manually on all files
pre-commit run --all-files

# Run specific hook
pre-commit run black --all-files

# Skip hooks for a commit (use sparingly)
git commit --no-verify -m "message"

# Run manual C++ coverage check
pre-commit run cpp-coverage-check --hook-stage manual
```

**Note:** C++ coverage check is configured as a **manual hook** because it:
- Takes 5-15 minutes to run (rebuild + test + report generation)
- Would slow down every commit excessively
- Already runs automatically in CI/CD on every PR/push
- Available on-demand when developers want to check locally


### Coverage Tracking

**C++ Code Coverage** (Primary - 90% goal):
- Tool: gcov/lcov
- Workflow: `.github/workflows/coverage.yml`
- Local build: `cmake -DENABLE_COVERAGE=ON` then `make lcov`
- Reports generated in `build/lcov/html/`
- **Issue:** Create dedicated issue for C++ coverage infrastructure improvements

**Python Test Coverage** (Secondary - 80% goal):
- Tool: pytest-cov with coverage.py
- Command: `pytest --cov=tests --cov-report=html`
- Reports in `.local/coverage/`
- **Issue:** #8

**Other Languages:**
- Julia: Issue #9
- MATLAB/Octave: Issue #10

## Overview

JSBSim is a multi-platform Flight Dynamics Model (FDM) written in C++17. It's a physics and math model that simulates aircraft/rocket movement under applied forces and moments. The library can run standalone or integrate with simulation environments like Unreal Engine, FlightGear, and drone autopilot systems (ArduPilot, PX4).

## Build Commands

### Standard Build
```bash
# Create build directory
mkdir -p build && cd build

# Configure with CMake
cmake ..

# Build (use -j4 for parallel build with 4 cores)
make -j4

# The executable will be in build/src/JSBSim
```

### Build Variants

**Debug build:**
```bash
cmake -DCMAKE_CXX_FLAGS_DEBUG="-g -Wall" -DCMAKE_C_FLAGS_DEBUG="-g -Wall" -DCMAKE_BUILD_TYPE=Debug ..
make
```

**Release build with optimizations:**
```bash
cmake -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

**Build with shared libraries:**
```bash
cmake -DBUILD_SHARED_LIBS=ON ..
make
```

**Build with system Expat library (instead of bundled):**
```bash
cmake -DSYSTEM_EXPAT=ON ..
make
```

### Python Module Build

Requires Cython, Python 3 development headers, and (for tests) numpy, pandas, scipy.

```bash
# CMake automatically detects Cython and builds Python module
cmake -DBUILD_PYTHON_MODULE=ON ..
make

# To install Python module with CMake
cmake -DINSTALL_JSBSIM_PYTHON_MODULE=ON ..
make install

# Or install manually using pip (from build/python directory)
cd build/python
pip install build
python -m build --wheel
pip install jsbsim --no-index -f dist
```

## Testing

### Run All Tests
```bash
# From build directory
ctest -j4
```

### Run Specific Tests
```bash
# Run tests matching a pattern
ctest -R TestDensityAltitude

# Run tests containing "Altitude"
ctest -R Altitude

# Exclude tests containing "Altitude"
ctest -E Altitude

# Run specific test numbers (e.g., tests 12-14)
ctest -I 12,14
```

### Shared Library Tests

If tests fail with `ImportError: libJSBSim.so.1: cannot open shared object file`, set the library path:

```bash
# One-time for current command
LD_LIBRARY_PATH=$PWD/src ctest

# Set for entire session (from build directory)
export LD_LIBRARY_PATH=$PWD/src:$LD_LIBRARY_PATH
```

### Test Logs
Test output is in `build/Testing/Temporary/LastTestsFailed.log`

### Unit Tests

C++ unit tests use CxxTest framework. Sources are in `tests/unit_tests/`. Python integration tests are in `tests/` (e.g., `TestTurbine.py`, `TestActuator.py`).

## Documentation

```bash
# Generate Doxygen API documentation (requires Doxygen and Graphviz)
cd build
make doc

# Output will be in build/documentation/html
```

Online documentation:
- C++ API: https://jsbsim-team.github.io/jsbsim/
- Reference Manual: https://jsbsim-team.github.io/jsbsim-reference-manual

## High-Level Architecture

### Core Executive (FGFDMExec)

`src/FGFDMExec.{h,cpp}` is the main executive class. All simulation components are instantiated, initialized, and run through this class. Typical usage:

```cpp
JSBSim::FGFDMExec fdm;
fdm.LoadScript("path/to/script.xml");
fdm.RunIC();
while (fdm.Run()) { /* simulation loop */ }
```

### Model System

The simulation is organized into models under `src/models/`:

- **FGPropagate**: Integration of equations of motion, position, velocity, orientation
- **FGAccelerations**: Computes accelerations (linear and angular)
- **FGAtmosphere**: Atmospheric models (ISA 1976, MSIS, etc.) in `models/atmosphere/`
- **FGAerodynamics**: Aerodynamic forces and moments
- **FGPropulsion**: Engine/propulsion systems in `models/propulsion/`
- **FGFCS**: Flight control system in `models/flight_control/`
- **FGMassBalance**: Mass properties, inertia, CG calculations
- **FGGroundReactions**: Landing gear, ground contact forces (FGLGear)
- **FGInertial**: Reference frame transformations, gravitational model
- **FGAuxiliary**: Auxiliary/derived parameters (e.g., Mach, airspeed)
- **FGAircraft**: Aircraft-specific configuration
- **FGBuoyantForces**: Buoyancy for lighter-than-air vehicles (FGGasCell)
- **FGExternalReactions**: External forces/moments applied to aircraft

Models inherit from `FGModel` base class which provides the framework for initialization and execution ordering.

### Input/Output System

Located in `src/input_output/`:

- **FGXMLElement**: XML parsing and representation (uses Expat library)
- **FGScript**: Script file execution for automated simulations
- **FGPropertyManager**: Property tree management (based on SimGear property system)
- **Output types**: FGOutputFile, FGOutputSocket, FGOutputTextFile, FGOutputFG (FlightGear)
- **Input types**: FGInputSocket, FGUDPInputSocket
- **FGfdmSocket**: Network socket abstraction

### Math Utilities

`src/math/` contains mathematical utilities:
- Vector/matrix operations
- Quaternion math (FGQuaternion)
- Table lookups (FGTable)
- Function/condition evaluation (FGFunction, FGCondition, FGParameter)
- Template functions (FGTemplateFunc)
- Location/coordinate transformations (FGLocation)

### Initialization

`src/initialization/` handles initial conditions (FGInitialCondition) and trimming (FGTrim, FGLinearization).

### Property System

JSBSim uses a hierarchical property tree (from SimGear) accessible via string paths like `"position/h-sl-ft"` or `"velocities/vc-kts"`. Properties are the primary interface for:
- Getting/setting simulation state
- Configuring aircraft parameters
- Defining output channels
- Scripting conditions and events

### Aircraft Configuration

Aircraft are defined via XML files in `aircraft/` directory with sections for:
- Metrics (dimensions, wing area, reference points)
- Mass_balance (weights, CG, inertia)
- Ground_reactions (landing gear)
- Propulsion (engines, fuel tanks)
- Flight_control (autopilot, control laws)
- Aerodynamics (force/moment coefficients)
- External_reactions
- System (custom systems)
- Output (data logging)

### Script Execution

Scripts (`scripts/*.xml`) define automated simulation runs with:
- Initial conditions
- Events triggered by conditions
- Property modifications over time
- Output configuration

## Language and Standards

- **Language**: C++17
- **No external dependencies** (except optional system Expat)
- Bundled dependencies: Expat (XML parsing), GeographicLib (geodesy), SimGear components (properties, I/O)
- **Compilation**: Standard C++17 compiler required (GCC, Clang, MSVC)

## Bindings and Interfaces

- **Python module**: Cython-based (`python/jsbsim.pyx.in`)
- **MATLAB S-Function**: In `matlab/`
- **Unreal Engine plugin**: In `UnrealEngine/`
- **Julia package**: In `julia/` (optional, requires BUILD_JULIA_PACKAGE=ON)

## Running JSBSim

### Command Line

```bash
# Run a script
./build/src/JSBSim --script=scripts/c1721.xml

# See all options
./build/src/JSBSim --help
```

### Python Example

```python
import jsbsim

fdm = jsbsim.FGFDMExec(None)  # Use default aircraft data
fdm.load_script('scripts/c1723.xml')
fdm.run_ic()

while fdm.run():
    pass
```

## Development Workflow

1. Make code changes in `src/`
2. Build with `make` from `build/` directory
3. Run tests with `ctest` to verify changes
4. For new features, add unit tests in `tests/unit_tests/` (C++) or `tests/` (Python)
5. Update documentation in code comments (Doxygen format)

## Common Issues

**Tests fail with library not found**: Export `LD_LIBRARY_PATH=$PWD/src` from build directory before running ctest.

**CMake can't find Cython/Python**: Install Python 3 development headers and Cython (`pip install cython`).

**Linking errors on Windows**: Use `-DBUILD_SHARED_LIBS=ON` to avoid symbol conflicts, or ensure proper static library configuration.
