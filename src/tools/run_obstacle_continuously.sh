#!/bin/bash

# Navigate to the directory where this script is located
cd "$(dirname "$0")"

echo "Starting python3 -m src.obstacle_challenge.main_v5 in a continuous loop..."
echo "Press Ctrl+C to terminate the loop."

while true; do
    python3 -m src.obstacle_challenge.main_v5
done
