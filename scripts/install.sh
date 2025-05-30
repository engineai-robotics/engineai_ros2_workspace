#!/bin/bash

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"

# Remote host information list
declare -A HOST_CONFIGS

# Host 1 configuration (default)
HOST_CONFIGS["nezha,HOST_NAME"]="nezha"
HOST_CONFIGS["nezha,REMOTE_HOST"]="192.168.0.163"
HOST_CONFIGS["nezha,REMOTE_USER"]="user"
HOST_CONFIGS["nezha,REMOTE_PASSWORD"]="1"
HOST_CONFIGS["nezha,REMOTE_PATH"]="/home/user/apps"

# Host 2 configuration
HOST_CONFIGS["orin,HOST_NAME"]="orin"
HOST_CONFIGS["orin,REMOTE_HOST"]="192.168.0.162"
HOST_CONFIGS["orin,REMOTE_USER"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PASSWORD"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PATH"]="/home/ubuntu/apps"

# Local install directory
LOCAL_INSTALL_DIR="$root_dir/install"

# Default host is nezha
TARGET_HOST="nezha"

# Parse command line arguments - only accept host name
if [[ -n "$1" ]]; then
    TARGET_HOST="$1"
fi

# Check if target host exists
if [[ -z "${HOST_CONFIGS["$TARGET_HOST,REMOTE_HOST"]}" ]]; then
    echo "Error: Unknown target host '$TARGET_HOST'"
    echo "Available remote hosts:"
    for key in "${!HOST_CONFIGS[@]}"; do
        if [[ $key == *,HOST_NAME ]]; then
            host_name=${key%,HOST_NAME}
            echo "  $host_name: ${HOST_CONFIGS["$host_name,HOST_NAME"]} (${HOST_CONFIGS["$host_name,REMOTE_HOST"]})"
        fi
    done
    exit 1
fi

# Set remote host information
HOST_NAME="${HOST_CONFIGS["$TARGET_HOST,HOST_NAME"]}"
REMOTE_HOST="${HOST_CONFIGS["$TARGET_HOST,REMOTE_HOST"]}"
REMOTE_USER="${HOST_CONFIGS["$TARGET_HOST,REMOTE_USER"]}"
REMOTE_PASSWORD="${HOST_CONFIGS["$TARGET_HOST,REMOTE_PASSWORD"]}"
REMOTE_PATH="${HOST_CONFIGS["$TARGET_HOST,REMOTE_PATH"]}"

echo "Using remote host: $HOST_NAME ($REMOTE_HOST)"

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y sshpass
fi

# Check if local install directory exists
if [ ! -d "$LOCAL_INSTALL_DIR" ]; then
    echo "Error: Local install directory not found!"
    exit 1
fi

echo "Deploying install folder to remote host..."

# Create remote directory (if it doesn't exist)
sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" "mkdir -p $REMOTE_PATH"

# Use rsync to synchronize files
sshpass -p "$REMOTE_PASSWORD" rsync -avz --delete \
    "$LOCAL_INSTALL_DIR/" \
    "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/install/"

if [ $? -eq 0 ]; then
    echo "Deployment successful!"
    echo "To use the deployed package, on the remote machine run:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $REMOTE_PATH/install/setup.bash"
else
    echo "Deployment failed!"
    exit 1
fi
