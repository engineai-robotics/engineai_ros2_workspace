#!/bin/bash

# Script to extract engineai_robotics_third_party_libs.tar.gz to /opt directory

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Path to the compressed file (relative to script directory)
TAR_FILE="${SCRIPT_DIR}/engineai_robotics_third_party_libs.tar.gz"

# Check if the file exists
if [ ! -f "$TAR_FILE" ]; then
    echo "Error: File $TAR_FILE not found in current directory"
    exit 1
fi

# Check if script is running with sudo privileges
if [ "$(id -u)" -ne 0 ]; then
    echo "This script requires sudo privileges to extract to /opt"
    echo "Executing with sudo..."
    sudo "$0" "$@"
    exit $?
fi

echo "Extracting $TAR_FILE to /opt..."

# Extract the tar.gz file to /opt
tar -xzf "$TAR_FILE" -C /opt

if [ $? -eq 0 ]; then
    echo "Extraction completed successfully"
    echo "Third-party libraries have been installed to /opt"
else
    echo "Error: Failed to extract the file"
    exit 1
fi

exit 0