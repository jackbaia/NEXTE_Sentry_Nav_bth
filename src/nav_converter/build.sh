#!/bin/bash

# Build script for nav_converter package
# This script builds the package and provides helpful feedback

echo "=========================================="
echo "Building nav_converter package..."
echo "=========================================="

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

echo "Workspace directory: $WORKSPACE_DIR"
echo "Package directory: $SCRIPT_DIR"

# Check if we're in a catkin workspace
if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    echo "Error: Not in a catkin workspace or workspace not built"
    echo "Please run 'catkin_make' in the workspace root first"
    exit 1
fi

# Change to workspace directory
cd "$WORKSPACE_DIR"

# Build the package
echo "Building package..."
if catkin_make --pkg nav_converter; then
    echo "=========================================="
    echo "Build successful!"
    echo "=========================================="
    
    # Source the workspace
    source devel/setup.bash
    
    echo "Workspace sourced successfully"
    echo ""
    echo "To test the converter:"
    echo "1. roslaunch nav_converter nav_converter.launch"
    echo "2. rosrun nav_converter test_converter.py"
    echo ""
    echo "To integrate with navigation:"
    echo "Add this line to your navigation launch file:"
    echo "  <include file=\"\$(find nav_converter)/launch/nav_converter.launch\"/>"
    
else
    echo "=========================================="
    echo "Build failed!"
    echo "=========================================="
    echo "Please check the error messages above"
    exit 1
fi 