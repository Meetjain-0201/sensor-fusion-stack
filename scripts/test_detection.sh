#!/bin/bash

echo "=== Testing Camera Detection System ==="
echo ""
echo "Checking topics..."
ros2 topic list | grep -E "(camera|detection)"
echo ""
echo "Waiting for detections..."
timeout 5 ros2 topic echo /camera/detections --once
echo ""
echo "Detection system test complete!"
