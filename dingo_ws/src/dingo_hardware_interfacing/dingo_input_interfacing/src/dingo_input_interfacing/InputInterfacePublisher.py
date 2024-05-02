#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist

class InputInterfacePublisher:
    def __init__(self, config):
        self.config = config
        rospy.Subscriber("joy", Joy, self.input_callback)
        self.publisher = rospy.Publisher("input_interface_values", Twist, queue_size=10)

    def input_callback(self, msg):
        # Extract values from the Joy message
        horizontal_velocity_x = msg.axes[1] * self.config.max_x_velocity
        horizontal_velocity_y = msg.axes[0] * self.config.max_y_velocity
        yaw_rate = msg.axes[3] * self.config.max_yaw_rate
        pitch = msg.axes[4] * self.config.max_pitch
        height_movement = msg.axes[7]
        roll_movement = -msg.axes[6]
        height = 0.0  # Example value, replace with actual calculation
        roll = 0.0  # Example value, replace with actual calculation
        trot_event = bool(msg.buttons[5])
        hop_event = bool(msg.buttons[0])
        joystick_control_event = bool(msg.buttons[4])

        # Create a Twist message and fill in the values
        input_msg = Twist()
        input_msg.linear.x = horizontal_velocity_x
        input_msg.linear.y = horizontal_velocity_y
        input_msg.angular.z = yaw_rate

        # Publish the message
        self.publisher.publish(input_msg)

    def publish_internal_values(self):
        rospy.loginfo("Publishing internal values...")
        rospy.spin()  # Keep the node running

def main():
    rospy.init_node("input_interface_publisher")
    config = None  # Provide the configuration object
    publisher = InputInterfacePublisher(config)
    publisher.publish_internal_values()

if __name__ == "__main__":
    main()