#!/usr/bin/env bash
set -e

echo "=== Installing TrajSim2D ==="

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Repository root: ${ROOT_DIR}"

# -----------------------------
# Install core library in editable mode
# -----------------------------
echo "Installing trajsim2d_core (editable)..."

# Use --user to avoid system site-packages
pip install --upgrade pip setuptools wheel
pip install -e "${ROOT_DIR}" --user

# -----------------------------
# Make standalone examples executable
# -----------------------------
echo "Making standalone examples executable..."
chmod +x "${ROOT_DIR}"/standalone_examples/*.py || true

echo
echo "=== Installation complete ==="
echo "Standalone examples can now be run from command line."