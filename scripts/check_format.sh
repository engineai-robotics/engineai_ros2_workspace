#!/bin/bash
set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Workspace directory is one level up from the script directory
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Directories to check
CHECK_DIRS=("${WORKSPACE_DIR}/src" "${WORKSPACE_DIR}/scripts")

# Directories to ignore
IGNORE_DIRS="lcm_data"

# Function to perform C/C++ format check operation
cpp_format_check() {
    echo "Executing C/C++ format check operation..."
    
    # Process each directory
    for dir in "${CHECK_DIRS[@]}"; do
        if [ ! -d "$dir" ]; then
            echo "Warning: Directory does not exist: $dir, skipping C/C++ format check for this directory."
            continue
        fi
        
        echo "Checking C/C++ files in: $dir"
        
        # Run clang-format on all source files
        find "$dir" \( -name "$IGNORE_DIRS" -prune \) -o \( -type f \( \
            -name "*.cpp" -o \
            -name "*.cc" -o \
            -name "*.cxx" -o \
            -name "*.c" -o \
            -name "*.h" -o \
            -name "*.hpp" -o \
            -name "*.hh" -o \
            -name "*.hxx" \
        \) \) -print0 | xargs -0 /usr/bin/clang-format -style=file -i
        
        # Check if files are properly formatted
        while IFS= read -r -d '' file; do
            if ! git diff --exit-code -- "$file" ; then
                echo "Error: $file is not formatted correctly."
                exit 1
            fi
        done < <(find "$dir" \( ! -path "$dir/$IGNORE_DIRS/*" \) -type f \( \
            -name "*.cpp" -o \
            -name "*.cc" -o \
            -name "*.cxx" -o \
            -name "*.c" -o \
            -name "*.h" -o \
            -name "*.hpp" -o \
            -name "*.hh" -o \
            -name "*.hxx" \
        \) -print0)
    done
    
    echo "C/C++ format check operation completed."
}

yaml_format_check() {
    echo "Executing YAML format check operation..."
    
    # Process each directory
    for dir in "${CHECK_DIRS[@]}"; do
        if [ ! -d "$dir" ]; then
            echo "Warning: Directory does not exist: $dir, skipping YAML format check for this directory."
            continue
        fi
        
        echo "Checking YAML files in: $dir"
        
        # Run yamllint on all yaml files in the directory
        find "$dir" -type f -name "*.yaml" | while read -r yaml_file; do
            # Run yamllint and capture results
            yamllint --no-warnings -c "${WORKSPACE_DIR}/tools/scripts/ci/config/.yamllint" -f auto "$yaml_file"
            if [ $? -ne 0 ]; then
                echo "Yamllint failed on $yaml_file"
                exit 1
            fi
        done
    done
    
    echo "YAML format check operation completed."
}

python_format_check() {
    echo "Executing Python format check operation..."
    
    # Process each directory
    for dir in "${CHECK_DIRS[@]}"; do
        if [ ! -d "$dir" ]; then
            echo "Warning: Directory does not exist: $dir, skipping Python format check for this directory."
            continue
        fi
        
        echo "Checking Python files in: $dir"
        
        # Execute autopep8 check
        autopep8 --diff --exit-code --exclude lcm_msgs -r "$dir"
        
        # Check command execution result
        if [ $? -ne 0 ]; then
            echo "Error: autopep8 encountered an issue in $dir."
            exit 1
        fi
    done
    
    echo "Python format check operation completed."
}

# Execute all format checks
cpp_format_check
yaml_format_check
python_format_check