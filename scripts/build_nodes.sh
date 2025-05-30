#!/bin/bash
# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"

# Define node lists for different hosts
# Nodes to build on example
EXAMPLE_NODES=(
    "interface_protocol"
    "rdk_framework"
    "interface_example"
)

# Nodes to build on Orin
APPLICATION_NODES=(
    "interface_protocol"
    "rdk_framework"
    "robot_manager"
    "sound_hub"
    "system_monitor"
)

SIM_NODES=(
    "mujoco_simulator"
    "interface_protocol"
    "interface_example"
)
# Default target host is example
TARGET_HOST="example"

# Default build type is Release
BUILD_TYPE="Release"

# Parse command line arguments - only accept host name
if [[ -n "$1" ]]; then
    TARGET_HOST="$1"
fi

# Validate target host
if [[ "$TARGET_HOST" != "example" && "$TARGET_HOST" != "app" && "$TARGET_HOST" != "sim" ]]; then
    echo "Error: Invalid target host '$TARGET_HOST'"
    echo "Available hosts: example, app, sim"
    exit 1
fi

# Select the appropriate node list based on target host
if [[ "$TARGET_HOST" == "example" ]]; then
    NODES=("${EXAMPLE_NODES[@]}")
    echo "Building for example nodes"
elif [[ "$TARGET_HOST" == "app" ]]; then
    NODES=("${APPLICATION_NODES[@]}")
    echo "Building for application nodes"
elif [[ "$TARGET_HOST" == "sim" ]]; then
    NODES=("${SIM_NODES[@]}")
    echo "Building for simulation nodes"
fi

echo "Build type: $BUILD_TYPE"

# Create packages argument for colcon build
PACKAGES_ARG=""
if [[ ${#NODES[@]} -gt 0 ]]; then
    PACKAGES_ARG="--packages-select ${NODES[*]}"
fi

# Create build directory if it doesn't exist
mkdir -p "$root_dir/build"

# Run colcon build with the selected options
echo "Running build with the following nodes: ${NODES[*]}"
cd "$root_dir" && \
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --build-base build \
    --install-base install \
    $PACKAGES_ARG

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "To use the built packages, run:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $root_dir/install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi
