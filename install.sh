#!/bin/bash

# Define the source directory and the destination directory
destination_dir="/usr/local/bin/robot/py"

# Loop through every folder in the source directory
for folder in ./*; do
    if [ -d "$folder" ]; then
        # If the item is a directory, loop through Python files in that directory
        for python_file in "$folder"/*.py; do
            if [ -f "$python_file" ]; then
                # If it's a Python file, copy it to the destination directory
                cp "$python_file" "$destination_dir"
                echo "Copied: $python_file"
            fi
        done
    fi
done
