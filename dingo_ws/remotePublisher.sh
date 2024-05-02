#!/bin/bash

# Get the current IP address
ip_address=$(hostname -I | awk '{print $1}')

# Set ROS_MASTER_URI and ROS_IP
export ROS_MASTER_URI="http://$ip_address:11311"
export ROS_IP="$ip_address"

# Optional: Print out the set values for verification
echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"
echo "copy the following and paste it in the subscribing computer's terminal:\n"
echo "export ROS_MASTER_URI=http://$ip_address:11311\n"