#!/usr/bin/env bash
# Lang2Sim One-Command Quickstart & Demo Launcher
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

echo "========================================================"
echo "          🚀 Lang2Sim - Autonomous 3D Simulation         "
echo "========================================================"
echo ""

# 1. Check Docker
if ! command -v docker &> /dev/null; then
    echo "⚠️ Docker is not installed. Please install Docker to run Gazebo Harmonic simulations."
    echo "   Visit: https://docs.docker.com/engine/install/"
else
    if ! docker info &> /dev/null; then
        echo "⚠️ Docker daemon is not running. Starting Docker..."
        sudo systemctl start docker 2>/dev/null || true
    fi
    echo "📦 Checking simulation image (ghcr.io/duttaneel07/lang2sim-sim:staging)..."
    if ! docker image inspect ghcr.io/duttaneel07/lang2sim-sim:staging &>/dev/null; then
        echo "⏳ Pulling simulation image (this happens once, ~4GB)..."
        docker pull ghcr.io/duttaneel07/lang2sim-sim:staging
    else
        echo "✅ Simulation image is already cached locally."
    fi
fi

# 2. Install dependencies
echo ""
echo "📦 Installing Python package and dependencies..."
python3 -m pip install -q -e .

# 3. Allow X11 for local Gazebo GUI windows if running desktop Linux
if [ -n "$DISPLAY" ]; then
    xhost +local:root &>/dev/null || xhost + &>/dev/null || true
fi

echo ""
echo "✨ Starting Lang2Sim TUI Workspace..."
echo "========================================================"
exec python3 -m lang2sim.cli.app "$@"
