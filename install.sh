#!/bin/bash

# Define the source directory and the destination directory
destination_dir="/usr/local/bin/robot/py"
# Use the find command to locate all Python files in the source directory and its subdirectories
find . -type f -name "*.py" -exec cp {} "$destination_dir" \;

echo "Copied all python files to $destination_dir"