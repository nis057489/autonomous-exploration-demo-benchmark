#!/bin/bash

# Define the service name
SERVICE_NAME="/sound"

echo "Checking for ROS 2 service: $SERVICE_NAME..."

# Loop until the service is found in the ros2 service list
until ros2 service list | grep -q "^$SERVICE_NAME$"; do
    echo "Waiting for $SERVICE_NAME service to become available..."
    sleep 2
done

echo "Success: $SERVICE_NAME service is up and running!"
echo "Playing sound sequence..."

# Call the sound service
ros2 service call /sound turtlebot3_msgs/srv/Sound "value: 1"
