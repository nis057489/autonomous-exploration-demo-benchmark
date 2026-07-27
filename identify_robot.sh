#!/bin/bash

# Configuration
SERVICE_NAME="/sound"
MODEL_NAME="waffle_pi"

echo "Checking if ROS 2 network is active..."

# 1. Check if the sound service is already running
if ros2 service list 2>/dev/null | grep -q "^$SERVICE_NAME$"; then
    echo "Bringup is already running. Proceeding to play sound."
else
    echo "Bringup not detected. Starting TurtleBot 3 bringup in the background..."
    
    # Export environment variable and launch bringup in the background
    export TURTLEBOT3_MODEL=$MODEL_NAME
    ros2 launch turtlebot3_bringup robot.launch.py > /tmp/tb3_bringup.log 2>&1 &
    
    # Save the background process ID (PID)
    BRINGUP_PID=$!
    echo "Bringup started with PID: $BRINGUP_PID (Logs saved to /tmp/tb3_bringup.log)"
fi

# 2. Wait for the service to become available
echo "Waiting for $SERVICE_NAME service to become available..."
until ros2 service list 2>/dev/null | grep -q "^$SERVICE_NAME$"; do
    # Check if the background process died early due to an error
    if [ -n "$BRINGUP_PID" ] && ! kill -0 $BRINGUP_PID 2>/dev/null; then
        echo "ERROR: Background bringup process failed to start. Check /tmp/tb3_bringup.log"
        exit 1
    fi
    sleep 1
done

echo "Success: $SERVICE_NAME service is active!"
echo "Playing sound sequence..."

# 3. Call the sound service
ros2 service call /sound turtlebot3_msgs/srv/Sound "value: 1"
