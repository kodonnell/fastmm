#!/bin/bash
# Script to test cibuildwheel locally on Linux

set -e  # Exit on error

echo "=== Testing cibuildwheel locally ==="
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "Error: Docker is not running. Please start Docker and try again."
    exit 1
fi

# Install cibuildwheel if not already installed
if ! command -v cibuildwheel &> /dev/null; then
    echo "Installing cibuildwheel..."
    pip install cibuildwheel
fi

# Set platform to Linux
export CIBW_PLATFORM=linux

# Optional: Build only one Python version for faster testing
# Uncomment to test with just Python 3.11
# export CIBW_BUILD="cp311-*"

# Optional: Enable verbose output
# export CIBW_BUILD_VERBOSITY=3

echo "Starting cibuildwheel build for Linux..."
echo "This will use Docker to build manylinux wheels."
echo ""

# Run cibuildwheel
cibuildwheel --platform linux

echo ""
echo "=== Build complete ==="
echo "Wheels are in: ./wheelhouse/"
ls -lh ./wheelhouse/
