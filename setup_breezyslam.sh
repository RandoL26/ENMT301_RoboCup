#!/bin/bash
# setup_breezyslam.sh - Install BreezySLAM for RoboCup project
# 
# This script handles BreezySLAM installation on macOS ARM64
# The official BreezySLAM has ARM64 compatibility issues with C extensions
# This script builds and installs it correctly

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BREEZYSLAM_DIR="${PROJECT_DIR}/tools/BreezySLAM"

echo "🤖 RoboCup BreezySLAM Setup"
echo "============================"
echo ""

# Check if BreezySLAM already installed
if python3 -c "from breezyslam.algorithms import RMHC_SLAM" 2>/dev/null; then
    echo "✅ BreezySLAM already installed"
    exit 0
fi

# Check if BreezySLAM repo exists locally
if [ ! -d "$BREEZYSLAM_DIR" ]; then
    echo "📥 Cloning BreezySLAM from GitHub..."
    cd "${PROJECT_DIR}/tools"
    git clone https://github.com/simondlevy/BreezySLAM.git
    cd - > /dev/null
else
    echo "✓ BreezySLAM repo already present at $BREEZYSLAM_DIR"
fi

# Build and install
echo "🔨 Building BreezySLAM for ARM64 macOS..."
cd "${BREEZYSLAM_DIR}/python"

# Use ARCHFLAGS to handle arm64 compilation
export ARCHFLAGS=-Wno-error=unused-command-line-argument-hard-error-in-future
python3 setup.py build_ext --inplace --quiet

echo "📦 Installing BreezySLAM..."
python3 -m pip install -e . --quiet

cd - > /dev/null

# Verify installation
if python3 -c "from breezyslam.algorithms import RMHC_SLAM; print('✓ BreezySLAM import OK')" 2>/dev/null; then
    echo "✅ BreezySLAM installed successfully!"
    echo ""
    echo "Next steps:"
    echo "  1. Connect Teensy 4.0 to USB"
    echo "  2. Upload firmware: pio run -t upload"
    echo "  3. Run SLAM processor:"
    echo "     python3 tools/run_breezyslam.py --port /dev/cu.wchusbserial --baud 115200"
else
    echo "❌ BreezySLAM installation verification failed"
    exit 1
fi
