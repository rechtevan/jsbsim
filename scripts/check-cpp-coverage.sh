#!/bin/bash
# check-cpp-coverage.sh
#
# Manual pre-commit hook for C++ code coverage check.
# Run with: pre-commit run cpp-coverage-check --hook-stage manual
#
# This script:
# 1. Builds JSBSim with coverage enabled
# 2. Runs C++ unit tests
# 3. Generates coverage report
# 4. Displays summary
# 5. Opens HTML report in browser

set -e

echo "=========================================="
echo "C++ Code Coverage Check"
echo "=========================================="
echo ""

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir -p build
fi

cd build

# Configure with coverage if not already configured
if [ ! -f "CMakeCache.txt" ] || ! grep -q "ENABLE_COVERAGE:BOOL=ON" CMakeCache.txt; then
    echo "Configuring CMake with coverage enabled..."
    cmake -DENABLE_COVERAGE=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF ..
fi

# Build
echo ""
echo "Building JSBSim with coverage..."
make -j$(nproc) 2>&1 | tail -5

# Run C++ unit tests
echo ""
echo "Running C++ unit tests..."
ctest -R Test1 --output-on-failure | tail -20

# Generate coverage report
echo ""
echo "Generating coverage report..."
make lcov 2>&1 | grep -E "Overall coverage rate|lines\.\.\.|functions\.\."

# Display summary
echo ""
echo "=========================================="
echo "Coverage Report Generated"
echo "=========================================="
echo ""
echo "HTML Report: build/lcov/html/index.html"
echo ""
echo "To view report:"
echo "  xdg-open build/lcov/html/index.html"
echo "  # or"
echo "  open build/lcov/html/index.html  (macOS)"
echo ""

# Extract coverage percentage
if [ -f "lcov/data/capture/all_targets.info" ]; then
    COVERAGE=$(lcov --summary lcov/data/capture/all_targets.info 2>&1 | grep "lines" | awk '{print $2}')
    echo "Current C++ Coverage: $COVERAGE"
    echo ""

    # Optional: Check if coverage meets minimum threshold
    COVERAGE_NUM=$(echo "$COVERAGE" | sed 's/%//')
    MIN_COVERAGE=24.0

    if (( $(echo "$COVERAGE_NUM < $MIN_COVERAGE" | bc -l) )); then
        echo "WARNING: Coverage is below minimum threshold of ${MIN_COVERAGE}%"
        echo "Current: ${COVERAGE_NUM}%"
        exit 1
    else
        echo "✅ Coverage meets minimum threshold (${MIN_COVERAGE}%)"
    fi
fi

cd ..
