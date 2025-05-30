
#!/bin/bash

# List of libraries to build
LIBS=("yaml-cpp" "eigen-3.4.0" "mujoco" "MNN")

# Specify CMake compile options and installation directories for each library
declare -A CMAKE_OPTIONS
DEFAULT_INSTALL_PATH="/opt/engineai_robotics_third_party"
INSTALL_PATH="${DEFAULT_INSTALL_PATH}"
set_cmake_options() {
    COMMON_OPTIONS="-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${INSTALL_PATH} -DCMAKE_PREFIX_PATH=${INSTALL_PATH}"
    CMAKE_OPTIONS["yaml-cpp"]="$COMMON_OPTIONS"
    CMAKE_OPTIONS["eigen-3.4.0"]="$COMMON_OPTIONS"
    CMAKE_OPTIONS["mujoco"]="$COMMON_OPTIONS"
    CMAKE_OPTIONS["MNN"]="$COMMON_OPTIONS"
}

# Build path
BUILD_DIR="build"

# Get number of CPU cores
NUM_CORES=$(nproc)

# Function to build all libraries
build_libraries() {
    echo "Building libraries..."
    set_cmake_options
    mkdir -p "$BUILD_DIR"
    for lib in "${LIBS[@]}"; do
        echo "Building $lib with $NUM_CORES cores..."
        mkdir -p "$BUILD_DIR/$lib"

        # Process other libraries using CMake
        CMAKE_OPTIONS_VAR="${CMAKE_OPTIONS[$lib]}"
        cd "$BUILD_DIR/$lib" || exit
        cmake "../../$lib" $CMAKE_OPTIONS_VAR && make -j"$NUM_CORES"
        cd - || exit
    done
}

# Function to install libraries
install_libraries() {
    local install_dir=$1
    
    echo "Installing libraries to ${install_dir}..."
    if [[ -d ${install_dir} ]]; then
        echo "Removing existing installation directory ${install_dir}..."
        if [[ "${install_dir}" == "${DEFAULT_INSTALL_PATH}" ]]; then
            sudo rm -rf ${install_dir}/*
        else
            rm -rf ${install_dir}/*
        fi
    else
        if [[ "${install_dir}" == "${DEFAULT_INSTALL_PATH}" ]]; then
            sudo mkdir -p ${install_dir}
        else
            mkdir -p ${install_dir}
        fi
    fi

    for lib in "${LIBS[@]}"; do
        echo "Installing $lib..."
        # Process other libraries using CMake
        cd "$BUILD_DIR/$lib" || exit
        if [[ "${install_dir}" == "${DEFAULT_INSTALL_PATH}" ]]; then
            sudo make install
        else
            make install
        fi
        cd - || exit
    done
}

# Execute operations based on input parameters
case "$1" in
build)
    build_libraries
    ;;
install)
    INSTALL_PATH="${DEFAULT_INSTALL_PATH}"
    set_cmake_options
    install_libraries "${INSTALL_PATH}"
    ;;
pack)
    # Create temporary installation directory
    TEMP_DIR="$(mktemp -d)"
    echo "Using temporary directory: ${TEMP_DIR}"
    
    # Set installation path to temporary directory
    INSTALL_PATH="${TEMP_DIR}/engineai_robotics_third_party"
    set_cmake_options
    
    # First build all libraries
    echo "Building libraries for packing..."
    build_libraries
    
    # Install to temporary directory
    install_libraries "${INSTALL_PATH}"
    
    # Compress files
    echo "Compressing libraries..."
    ARCHIVE_NAME="engineai_robotics_third_party_libs.tar.gz"
    tar -czf "${ARCHIVE_NAME}" -C "${TEMP_DIR}" engineai_robotics_third_party
    
    # Clean up temporary directory
    echo "Cleaning up temporary directory..."
    rm -rf "${TEMP_DIR}"
    
    echo "Package created: ${ARCHIVE_NAME}"
    ;;
clean)
    echo "Cleaning up..."
    rm -rf "$BUILD_DIR"
    ;;
*)
    if [[ "$1" == "" ]]; then
        build_libraries
    else
        echo "Usage: $0 {build|install|clean|pack}"
        exit 1
    fi
    ;;
esac
