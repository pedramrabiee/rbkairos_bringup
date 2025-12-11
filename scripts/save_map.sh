#!/bin/bash
# Save map from SLAM Toolbox
# Usage: ./save_map.sh <map_name>
# Example: ./save_map.sh demo_world

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <map_name>"
    echo "Example: $0 demo_world"
    exit 1
fi

MAP_NAME=$1
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MAPS_DIR="$SCRIPT_DIR/../maps"

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

echo "Saving map to: $MAPS_DIR/$MAP_NAME"

# Save map using nav2_map_server
# Map topic is /map (not namespaced under /rbkairos)
ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$MAP_NAME" -t /map

echo "Map saved successfully!"
echo "Files created:"
echo "  - $MAPS_DIR/${MAP_NAME}.yaml"
echo "  - $MAPS_DIR/${MAP_NAME}.pgm"
