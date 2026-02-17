#!/bin/bash
#
# Motor Control UI - Launch Script
#
# Simple script to start the Motor Control application
# Makes it easy to run from anywhere
#

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to application directory
cd "$SCRIPT_DIR"

# Activate virtual environment
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Creating..."
    python3 -m venv venv
    source venv/bin/activate
    pip install --only-binary :all: -r requirements.txt
else
    source venv/bin/activate
fi

# Run the application
echo "Starting Motor Control UI..."
python main.py
