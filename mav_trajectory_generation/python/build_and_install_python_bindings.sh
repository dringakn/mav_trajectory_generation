#!/usr/bin/env bash
#
# build_and_install.sh
#
# This script bootstraps Nanobind (if needed), configures and builds your
# catkin workspace, copies the compiled Python extension into your package’s
# python folder, and installs the package in editable mode with pip.
#

set -euo pipefail

##
## User‑configurable variables
##

# Absolute path to your catkin workspace root
CATKIN_WS="${HOME}/personal_ws"
# Name of your package (must match the folder under src/)
PACKAGE_NAME="mav_trajectory_generation"

##
## 0) Ensure externals/nanobind is available
##

# Where your package’s C++ lives
PKG_DIR="${CATKIN_WS}/src/${PACKAGE_NAME}/${PACKAGE_NAME}"
EXTERNALS_DIR="${PKG_DIR}/externals"
NANOBIND_DIR="${EXTERNALS_DIR}/nanobind"

echo "🔍 Checking for externals directory..."
if [ ! -d "${EXTERNALS_DIR}" ]; then
    echo "📁 Creating externals folder at ${EXTERNALS_DIR}"
    mkdir -p "${EXTERNALS_DIR}"
fi

echo "🔍 Checking for nanobind..."
if [ ! -d "${NANOBIND_DIR}" ]; then
    echo "🌱 Cloning Nanobind into ${NANOBIND_DIR}"
    git clone https://github.com/wjakob/nanobind.git "${NANOBIND_DIR}"
    echo "🔄 Initializing Nanobind submodules"
    (cd "${NANOBIND_DIR}" && git submodule update --init --recursive)
else
    echo "✅ Nanobind already present"
fi

##
## 1) Configure and build the workspace
##

echo "🔧 Configuring workspace"
cd "${CATKIN_WS}"
echo "🏗️  Building all packages"
catkin build --workspace "${CATKIN_WS}" --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5

##
## 2) Copy the compiled .so into your Python package
##

# Source .so location (wildcard to match the ABI tag)
SRC_SO="${CATKIN_WS}/devel/.private/${PACKAGE_NAME}/lib/mav_trajectory_generation_py.so"
# Destination folder inside your Python package
DST_DIR="${CATKIN_WS}/src/${PACKAGE_NAME}/${PACKAGE_NAME}/python/${PACKAGE_NAME}"

echo "📋 Ensuring destination exists: ${DST_DIR}"
mkdir -p "${DST_DIR}"

echo "📤 Copying compiled extension:"
echo "    from: ${SRC_SO}"
echo "      to: ${DST_DIR}/"
cp "${SRC_SO}" "${DST_DIR}/"

##
## 3) Install in editable (dev) mode
##

echo "🚀 Installing ${PACKAGE_NAME} in editable mode"
cd "${CATKIN_WS}/src/${PACKAGE_NAME}/${PACKAGE_NAME}"
pip install -e .

echo "✅ Done! Your package is installed and ready for development."
