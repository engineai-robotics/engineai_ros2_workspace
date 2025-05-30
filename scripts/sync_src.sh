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
HOST_CONFIGS["nezha,REMOTE_PATH"]="/home/user/source/engineai_workspace"

# Host 2 configuration
HOST_CONFIGS["orin,HOST_NAME"]="orin"
HOST_CONFIGS["orin,REMOTE_HOST"]="192.168.0.162"
HOST_CONFIGS["orin,REMOTE_USER"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PASSWORD"]="ubuntu"
HOST_CONFIGS["orin,REMOTE_PATH"]="/home/ubuntu/source/engineai_workspace"

# Files and directories to synchronize
SYNC_FILES=(
  "src"
  "scripts"
)

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

# Create remote directory (if it doesn't exist)
sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" "mkdir -p $REMOTE_PATH"

# Synchronize each file/directory in the list
echo "Synchronizing source files to remote host..."
for item in "${SYNC_FILES[@]}"; do
    local_path="$root_dir/$item"
    
    # Check if local path exists
    if [ ! -e "$local_path" ]; then
        echo "Warning: Local path '$local_path' does not exist, skipping..."
        continue
    fi
    
    echo "Syncing: $item"
    
    # Create parent directory on remote if needed
    remote_parent_dir=$(dirname "$item")
    if [ "$remote_parent_dir" != "." ]; then
        sshpass -p "$REMOTE_PASSWORD" ssh -o StrictHostKeyChecking=no "$REMOTE_USER@$REMOTE_HOST" "mkdir -p $REMOTE_PATH/$remote_parent_dir"
    fi
    
    # Use rsync to synchronize the file/directory
    # Adding trailing slash to source directory to copy contents rather than the directory itself
    if [ -d "$local_path" ]; then
        # For directories, add trailing slash to copy contents
        sshpass -p "$REMOTE_PASSWORD" rsync -avz --delete \
            "$local_path/" \
            "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/$item/"
    else
        # For files, use normal syntax
        sshpass -p "$REMOTE_PASSWORD" rsync -avz --delete \
            "$local_path" \
            "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/$item"
    fi
    
    if [ $? -ne 0 ]; then
        echo "Error: Failed to synchronize '$item'"
        exit 1
    fi
done

echo "Source synchronization completed successfully!"
echo "The Remote Path is: $REMOTE_PATH"
