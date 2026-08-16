#!/bin/bash

# Navigate to the repository root directory
cd "$(dirname "$0")/../.."

echo "Starting python3 -m src.obstacle_challenge.main -b in a continuous loop..."
echo "Press Ctrl+C to terminate the loop."

while true; do
    python3 -m src.obstacle_challenge.main -b -v
done
