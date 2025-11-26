#!/bin/bash
# Script to install rvo2 library for Python 3.11+
# This fixes the CMakeLists.txt issue in the original repository

set -e

echo "Installing rvo2 library..."

# Check if Python 3.11+ is available
python_version=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
required_version="3.11"

if [ "$(printf '%s\n' "$required_version" "$python_version" | sort -V | head -n1)" != "$required_version" ]; then
    echo "Error: Python 3.11+ is required. Current version: $python_version"
    exit 1
fi

# Install build dependencies
echo "Installing build dependencies..."
pip install Cython

# Check if cmake is available
if ! command -v cmake &> /dev/null; then
    echo "cmake not found. Installing via conda..."
    if command -v conda &> /dev/null; then
        conda install cmake -y
    else
        echo "Error: cmake is required but conda is not available."
        echo "Please install cmake manually: brew install cmake"
        exit 1
    fi
fi

# Clone and fix the repository
TMP_DIR=$(mktemp -d)
echo "Cloning Python-RVO2 repository to $TMP_DIR..."
git clone https://github.com/sybrenstuvel/Python-RVO2.git "$TMP_DIR/Python-RVO2"

# Fix CMakeLists.txt
echo "Fixing CMakeLists.txt..."
sed -i '' 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.5)/' "$TMP_DIR/Python-RVO2/CMakeLists.txt"

# Install the package
echo "Installing rvo2..."
pip install --no-build-isolation "$TMP_DIR/Python-RVO2"

# Cleanup
rm -rf "$TMP_DIR"

echo "✅ rvo2 installed successfully!"
python3 -c "import rvo2; print('Verification: rvo2 can be imported')"

