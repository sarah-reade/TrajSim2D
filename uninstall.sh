#!/usr/bin/env bash
set -e

echo "=== Uninstalling TrajSim2D core library ==="

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Repository root: ${ROOT_DIR}"

# -----------------------------
# Uninstall core library
# -----------------------------
# Works whether installed normally or editable (--user)
pip uninstall -y trajsim2d-core


echo "=== Uninstallation complete ==="
