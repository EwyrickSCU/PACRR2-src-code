#!/usr/bin/env python3

import rospy
import sys, signal
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
import threading
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from timer import timer
from sshkeyboard import listen_keyboard, stop_listening
import random


class AutoMove:
    def __init__(self):
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)
        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0., 0., 0., 0., 0., 0., 0., 0.]
        self.current_joy_message.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.state_message = rospy.Subscriber("dingo_state", Int32, self.state_callback)
        self.scan_message = rospy.Subscriber("/scan", LaserScan, self.laser_callback)



        self.odom_message = rospy.Subscriber("/dingoodom", Odometry, self.odom_callback)



        self.currentTrot = 0
        self.obstacleAverage = 100
        self.backObstacleAverage = 100
        self.leftObstacleAverage = 100
        self.rightObstacleAverage = 100
        self.disabled = False
        self.prev_time = None
        self.prev_linear_acceleration_x = None
        self.prev_linear_acceleration_y = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_th = 0.0
        

        self.distance_x = 0.0
        self.distance_y = 0.0
        self.prev_linear_velocity_x = None
        self.prev_linear_velocity_y = None



        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.counter = 0
        self.prev_distance_forward = None
        self.prev_distance_backward = None
        self.prev_distance_right = None
        self.prev_distance_left = None
        # self.start_keyboard_listener()
    
    def start_keyboard_listener(self):
        keyboard_thread = threading.Thread(target=self.start_listen_keyboard)
        keyboard_thread.daemon = True
        keyboard_thread.start()
    def start_listen_keyboard(self):
        listen_keyboard(on_press=self.on_press, on_release=self.on_release)

    def on_press(self,key):
        print("press detected")
        if key == 'w':
            stop_listening()
            self.enable()
            self.disabled = True
            self.current_joy_message = Joy()
            self.publish_current_command()
            sys.exit(0)
    def on_release(self, key):
        pass
    
    def odom_callback(self, msg):
        print("printing odoms")
        print(msg)


        self.x_position = msg.pose.pose.position.x
        self.y_postion = msg.pose.pose.position.x
        # print("x")
        # print(self.x_position)
        # pass

    
    def timeSleep(self, duration, dir="f"):
        with timer() as t:
            while t.elapse < duration:
                rospy.sleep(0.1)
                if dir == "f":
                    if self.obstacleAverage <1:
                        return
                elif dir == "b":
                    if self.backObstacleAverage <1:
                        return
            # print(t.elapse)

    
    def laser_callback(self, msg):
        # print("scanned")
        # print(msg.ranges)
        # print(msg.ranges[:20])

        # Detect obstacles ahead
        obstacleReadings = msg.ranges[558:578]
        obstacleSum = 0
        obstacleCount = 0
        for num in obstacleReadings:
            if num != "inf":
                num = float(num)
                if num>0.16 and num<12:
                    obstacleSum+=num
                    obstacleCount+=1
        if obstacleCount > 0:
            self.obstacleAverage = obstacleSum / obstacleCount

        # Detect obstacles behind
        obstacleReadings = msg.ranges[:3]
        obstacleReadings+=msg.ranges[1131:]
        obstacleSum = 0
        obstacleCount = 0
        for num in obstacleReadings:
            if num != "inf":
                num = float(num)
                if num>0.16 and num<12:
                    obstacleSum+=num
                    obstacleCount+=1
        if obstacleCount > 0:
            self.backObstacleAverage = obstacleSum / obstacleCount
        
        # Detect Obstacles to the left
        obstacleReadings = msg.ranges[834:854]
        obstacleSum = 0
        obstacleCount = 0
        for num in obstacleReadings:
            if num != "inf":
                num = float(num)
                if num>0.16 and num<12:
                    obstacleSum+=num
                    obstacleCount+=1
        if obstacleCount > 0:
            self.leftObstacleAverage = obstacleSum / obstacleCount

        # Detect Obstacles to the right
        obstacleReadings = msg.ranges[272:284]
        obstacleSum = 0
        obstacleCount = 0
        for num in obstacleReadings:
            if num != "inf":
                num = float(num)
                if num>0.16 and num<12:
                    obstacleSum+=num
                    obstacleCount+=1
        if obstacleCount > 0:
            self.rightObstacleAverage = obstacleSum / obstacleCount




    def state_callback(self, msg):
        self.currentTrot = msg.data
        # print("Trotting")
        # print(self.currentTrot)
        
    def move_right_for_duration(self, duration):
        msg = self.current_joy_message
        msg.axes[3] = -1.0
        self.current_joy_message = msg
        self.publish_current_command()
        self.timeSleep(duration, "r")  # Sleep for the specified duration
        msg.axes[3] = 0.0  # Stop movement after duration
        self.current_joy_message = msg
        self.publish_current_command()

    # def move_right_for_distance(self, ):
        #

    def move_left_for_duration(self, duration):
        msg = self.current_joy_message
        msg.axes[3] = 1.0
        self.current_joy_message = msg
        self.publish_current_command()
        self.timeSleep(duration, "r")  # Sleep for the specified duration
        msg.axes[3] = 0.0  # Stop movement after duration
        self.current_joy_message = msg
        self.publish_current_command()



    def move_forward_for_duration(self, duration):
        msg = self.current_joy_message
        msg.axes[1] = 0.7  # Assuming positive value moves forward
        self.current_joy_message = msg
        self.publish_current_command()
        self.timeSleep(duration, "f")  # Sleep for the specified duration
        msg.axes[1] = 0.0  # Stop movement after duration
        self.current_joy_message = msg
        self.publish_current_command()



    def move_backward_for_duration(self, duration):
        msg = self.current_joy_message
        msg.axes[1] = -0.8  # Assuming positive value moves forward
        self.current_joy_message = msg
        self.publish_current_command()
        self.timeSleep(duration, "b")  # Sleep for the specified duration
        msg.axes[1] = 0.0  # Stop movement after duration
        self.current_joy_message = msg
        self.publish_current_command()


    def enable(self):
        msg = self.current_joy_message
        msg.buttons[5] = 1  # Set button corresponding to '1' key to 1
        self.current_joy_message = msg
        self.publish_current_command()
        msg.buttons[5] = 0



    def publish_current_command(self):
        self.current_joy_message.header.stamp = rospy.Time.now()
        self.joystick_message_pub.publish(self.current_joy_message)

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    rospy.init_node("auto_input_listener")
    rate = rospy.Rate(30)
    rightOrLeft = True
    if os.getenv("DISPLY", default="-") != "-":
        rospy.logfatal("This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
        rospy.sleep(1)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    auto_mover = AutoMove()


    try:

        
        while not rospy.is_shutdown(): 
            if auto_mover.currentTrot == 0 and auto_mover.disabled == False:
                auto_mover.enable()
                # pass
            if auto_mover.disabled == False and auto_mover.currentTrot != 0:
                print("Front Distance")
                print(auto_mover.obstacleAverage )
                print("Back Distance")
                print(auto_mover.backObstacleAverage )
                print("Left Distance")
                print(auto_mover.rightObstacleAverage )
                print("Right Distance")
                print(auto_mover.leftObstacleAverage )
                # auto_mover.move_backward_for_duration(3)
                if auto_mover.obstacleAverage <0.5:
                    auto_mover.timeSleep(0.5)
                    print("Moving Backward")
                    # if auto_mover.backObstacleAverage > 1:
                    auto_mover.move_backward_for_duration(2)
                    turnAmount = random.randint(1, 4)
                    turnDirection = random.random()-0.5
                    print("turn amount")
                    print(turnAmount)
                    print("turnDirection")
                    print(turnDirection)
                    if turnDirection < 0:
                        auto_mover.move_left_for_duration(turnAmount)
                    else:
                        auto_mover.move_right_for_duration(turnAmount)

                    # auto_mover.move_right_for_duration(3)

                else:
                    print("Moving Forward")
                    auto_mover.move_forward_for_duration(2)
                # auto_mover.timeSleep(0.5)
                rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard node interrupted.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard node stopped by user.")
    finally:
        rospy.loginfo("Exiting...")

if __name__ == "__main__":
    main()

def main():
    rospy.init_node("auto_input_listener")
    rate = rospy.Rate(30)
    rightOrLeft = True
    if os.getenv("DISPLY", default="-") != "-":
        rospy.logfatal("This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
        rospy.sleep(1)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    auto_mover = AutoMove()
