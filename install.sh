#!/bin/bash
# install.sh — Autoware-Buggy Go-Kart Setup Script
# Run this from inside the cloned Autoware-Buggy repo after colcon build.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo " Autoware-Buggy Go-Kart Install Script"
echo "========================================="
echo "Repo root: $SCRIPT_DIR"
echo ""

# ── 1. Verify we're in the right place ───────────────────────────────────────
if [ ! -d "$SCRIPT_DIR/src" ]; then
  echo "[ERROR] src/ not found. Run this script from the Autoware-Buggy root."
  exit 1
fi

# ── 2. Copy gokart_sensor_kit_launch into src/ ───────────────────────────────
echo "[1/4] Copying gokart_sensor_kit_launch into src/..."
cp -r "$SCRIPT_DIR/gokart_packages/gokart_sensor_kit_launch" \
      "$SCRIPT_DIR/src/launcher/autoware_launch/sensor_kit/"
echo "       Done."

# ── 3. Copy gokart_vehicle_launch into src/ ──────────────────────────────────
echo "[2/4] Copying gokart_vehicle_launch into src/..."
cp -r "$SCRIPT_DIR/gokart_packages/gokart_vehicle_launch" \
      "$SCRIPT_DIR/src/launcher/autoware_launch/vehicle/"
echo "       Done."

# ── 4. Patch NDT scan matcher params ─────────────────────────────────────────
echo "[3/4] Patching NDT scan matcher params..."
NDT_PARAM="$SCRIPT_DIR/src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml"

if [ ! -f "$NDT_PARAM" ]; then
  echo "[WARN] NDT param file not found, skipping patch."
else
  sed -i 's/converged_param_nearest_voxel_transformation_likelihood: 2.3/converged_param_nearest_voxel_transformation_likelihood: 1.5/' "$NDT_PARAM"
  sed -i 's/num_threads: [0-9]*/num_threads: 8/' "$NDT_PARAM"
  echo "       converged_param_nearest_voxel_transformation_likelihood → 1.5"
  echo "       num_threads → 8"
fi

# ── 5. Make scripts executable ───────────────────────────────────────────────
echo "[4/4] Setting permissions on Buggyscripts/..."
chmod +x "$SCRIPT_DIR/Buggyscripts/launch_gokart.sh"
echo "       Done."

# ── 6. Rebuild affected packages ─────────────────────────────────────────────
echo ""
echo "Building gokart packages..."
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select \
    gokart_sensor_kit_description \
    gokart_sensor_kit_launch \
    gokart_vehicle_description \
    gokart_vehicle_launch

echo ""
echo "========================================="
echo " Install complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "  source $SCRIPT_DIR/install/setup.bash"
echo "  bash $SCRIPT_DIR/Buggyscripts/launch_gokart.sh"
