#!/usr/bin/env python3

# import rospy
# import sys, signal
# import os
# from sensor_msgs.msg import Joy
# from std_msgs.msg import Int32
# import threading
# import time
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from timer import timer
# from math import cos, tan, sin, atan, sqrt, acos
# import numpy as np
# from sshkeyboard import listen_keyboard, stop_listening
# import random
# from gazebo_msgs.msg import ModelStates
# import math
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseStamped
# from simple_pid import PID
# from contextlib import contextmanager


# class AutoMove:
#     def __init__(self):
#         self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)
#         self.current_joy_message = Joy()
#         self.current_joy_message.axes = [0., 0., 0., 0., 0., 0., 0., 0.]
#         self.current_joy_message.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#         self.state_message = rospy.Subscriber("dingo_state", Int32, self.state_callback)
#         # self.gazebo_message = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback )

#         # self.scan_message = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
#         # self.slam_message = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_callback)

#         self.odom_message = rospy.Subscriber("/wit/imu", Imu, self.odom_callback)
#         # self.odom_message = rospy.Subscriber("/imu/data", Imu, self.odom_callback)
#         # self.odom_message = rospy.Subscriber("/dingoodom", Odometry, self.odom_callback)



#         self.currentTrot = 0
#         self.obstacleAverage = 100
#         self.backObstacleAverage = 100
#         self.leftObstacleAverage = 100
#         self.rightObstacleAverage = 100
#         self.disabled = False
#         self.prev_time = None
#         self.prev_linear_acceleration_x = None
#         self.prev_linear_acceleration_y = None
#         self.velocity_x = 0.0
#         self.velocity_y = 0.0
#         self.velocity_th = 0.0
        

#         self.moveCounter = 0

#         self.distance_x = 0.0
#         self.distance_y = 0.0
#         self.prev_linear_velocity_x = None
#         self.prev_linear_velocity_y = None

#         self.test_var = 0.0

#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.counter = 0
#         self.prev_distance_forward = None
#         self.prev_distance_backward = None
#         self.prev_distance_right = None
#         self.prev_distance_left = None
#         # self.start_keyboard_listener()
#         self.roll_x = 0.0
#         self.pitch_y =0.0
#         self.yaw_z =0.0

#         # odom frame at start is 0,0,0
#         self.dt = 0.01  # Assuming a constant time step
#         self.vx = 0.0   # Initial velocity in x direction
#         self.vy = 0.0   # Initial velocity in y direction
#         self.vz = 0.0   # Initial velocity in z direction
#         self.px = 0.0    # Initial position in x direction
#         self.py = 0.0    # Initial position in y direction
#         self.pz = 0.0    # Initial position in z direction

#         # self.pid = PID(3.0,0.0, 3, setpoint=0.0)
#         self.pid = PID(5.0,0.7, 0.05, setpoint=0)
#         self.pid.output_limits = (-0.60, 0.60)


#     def euler_from_quaternion(self, x, y, z, w):
#         """
#         Convert a quaternion into euler angles (roll, pitch, yaw)
#         roll is rotation around x in radians (counterclockwise)
#         pitch is rotation around y in radians (counterclockwise)
#         yaw is rotation around z in radians (counterclockwise)
#         """
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll_x = math.atan2(t0, t1)
     
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch_y = math.asin(t2)
     
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw_z = math.atan2(t3, t4)
     
#         return roll_x, pitch_y, yaw_z # in radians

#     def odom_callback(self, msg):
#         # Extract orientation
#         self.x_quaternion = msg.orientation.x
#         self.y_quaternion = msg.orientation.y
#         self.z_quaternion = msg.orientation.z
#         self.w_quaternion = msg.orientation.w
#         self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x_quaternion, self.y_quaternion, self.z_quaternion, self.w_quaternion)
#         # print("x")
#         # print(self.roll_x)
#         # print(self.pitch_y)
#         # print(self.yaw_z)

#         self.test_var = self.test_var + 1
        
#         # Extract angular velocity
#         self.angular_velocity_x = msg.angular_velocity.x
#         self.angular_velocity_y = msg.angular_velocity.y
#         self.angular_velocity_z = msg.angular_velocity.z

#         # Extract linear acceleration
#         self.linear_acceleration_x = msg.linear_acceleration.x
#         self.linear_acceleration_y = msg.linear_acceleration.y
#         self.linear_acceleration_z = msg.linear_acceleration.z
        
#     def slam_callback(self, msg):
#         self.px = msg.pose.position.x
#         self.py = msg.pose.position.y
#         self.x_quaternion = msg.pose.orientation.x
#         self.y_quaternion = msg.pose.orientation.y
#         self.z_quaternion = msg.pose.orientation.z
#         self.w_quaternion = msg.pose.orientation.w
#         self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x_quaternion, self.y_quaternion, self.z_quaternion, self.w_quaternion)
#         self.yaw_z += 1.57
        
#     def timeSleep(self, duration, dir="f"):
#         with timer() as t:
#             while t.elapse < duration:
#                 rospy.sleep(0.1)
#                 if dir == "f":
#                     if self.obstacleAverage <1:
#                         return False
#                 elif dir == "b":
#                     if self.backObstacleAverage <1:
#                         return False

#     def state_callback(self, msg):
#         self.currentTrot = msg.data
    
#     def imu_position_integration(self):
#         # Extract linear accelerations
#         ax = self.linear_acceleration_x
#         ay = self.linear_acceleration_y
#         az = self.linear_acceleration_z
#         print("ax = ", ax, "| ay = ", ay, "| az = ", az)

#         # Integrate accelerations to obtain velocities using trapezoidal rule
#         self.vx += 0.5 * (ax + self.last_ax) * self.dt
#         self.vy += 0.5 * (ay + self.last_ay) * self.dt
#         self.vz += 0.5 * (az + self.last_az) * self.dt

#         # Integrate velocities to obtain positions using trapezoidal rule
#         self.px += 0.5 * (self.vx + self.last_vx) * self.dt
#         self.py += 0.5 * (self.vy + self.last_vy) * self.dt
#         self.pz += 0.5 * (self.vz + self.last_vz) * self.dt

#         # Store current accelerations for next iteration
#         self.last_ax = ax
#         self.last_ay = ay
#         self.last_az = az

#         # Store current velocities for next iteration
#         self.last_vx = self.vx
#         self.last_vy = self.vy
#         self.last_vz = self.vz

#     @contextmanager
#     def timer(self):
#         start = time.time()
#         class Timer:
#             @property
#             def elapse(self):
#                 return time.time() - start
#         yield Timer()

#     def move_forward_for_duration(self, duration):
#         msg = self.current_joy_message
#         msg.axes[1] = 0.5  # Assuming positive value moves forward
#         self.current_joy_message = msg
#         self.publish_current_command()

#         initial_yaw = self.yaw_z  # Capture the initial yaw value
#         self.pid.setpoint = initial_yaw  # Set the PID controller setpoint to the initial yaw

#         with self.timer() as t:
#             while t.elapse < duration:
#                 msg = self.current_joy_message

#                 # Calculate the correction factor using the PID controller
#                 correction = self.pid(self.yaw_z)
                
#                 with open('/home/pacrr-pi/pacrr-src-code/imuTest.txt', 'a') as file:
#                     file.write(f"{self.yaw_z}\n")

#                 with open('/home/pacrr-pi/pacrr-src-code/correction.txt', 'a') as file:
#                     file.write(f"{correction}\n")

#                 print("IMU: ", self.yaw_z)
#                 print("Correction: ", correction)
#                 msg.axes[3] = correction  # Apply the correction to the robot's direction

#                 self.current_joy_message = msg
#                 self.publish_current_command()

#                 rospy.sleep(0.05)
#                 if self.obstacleAverage < 1:
#                     return False

#         msg.axes[1] = 0.0  # Stop movement after duration
#         msg.axes[3] = 0.0
#         self.current_joy_message = msg
#         self.publish_current_command()


#     def enable(self):
#         msg = self.current_joy_message
#         msg.buttons[5] = 1  # Set button corresponding to '1' key to 1
#         self.current_joy_message = msg
#         self.publish_current_command()
#         msg.buttons[5] = 0
#         self.publish_current_command()

#     def publish_current_command(self):
#         self.current_joy_message.header.stamp = rospy.Time.now()
#         self.joystick_message_pub.publish(self.current_joy_message)

# def signal_handler(sig, frame):
#     sys.exit(0)

# def main():
#     rospy.init_node("auto_input_listener")
#     rate = rospy.Rate(30)
#     rightOrLeft = True
#     if os.getenv("DISPLY", default="-") != "-":
#         rospy.logfatal("This device does not have a display connected. The keyboard node requires a connected display due to a limitation of the underlying package. Keyboard node now shutting down")
#         rospy.sleep(1)
#         sys.exit(0)

#     signal.signal(signal.SIGINT, signal_handler)
#     auto_mover = AutoMove()


#     try:
#         auto_mover.timeSleep(7)

#         while not rospy.is_shutdown(): 
#             if auto_mover.currentTrot == 0 and auto_mover.disabled == False:
#                 auto_mover.enable()
#                 auto_mover.timeSleep(1)
#                 # pass
#             if auto_mover.disabled == False and auto_mover.currentTrot != 0:
#                 auto_mover.move_forward_for_duration(10)
#                 # auto_mover.move_straight_for_distance(0.5)
#                 while auto_mover.currentTrot != 0: 
#                     auto_mover.enable()
#                     auto_mover.disabled = True
#                     auto_mover.timeSleep(0.5)
                    
#                     rate.sleep()
                
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Keyboard node interrupted.")
#     except KeyboardInterrupt:
#         rospy.loginfo("Keyboard node stopped by user.")
#     finally:
#         rospy.loginfo("Exiting...")

# if __name__ == "__main__":
#     main()

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
from math import cos, tan, sin, atan, sqrt, acos
import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
import random
from gazebo_msgs.msg import ModelStates
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from simple_pid import PID
from contextlib import contextmanager


class AutoMove:
    def __init__(self):
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)
        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0., 0., 0., 0., 0., 0., 0., 0.]
        self.current_joy_message.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.state_message = rospy.Subscriber("dingo_state", Int32, self.state_callback)
        self.slam_message = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_callback)

        # self.odom_message = rospy.Subscriber("/wit/imu", Imu, self.odom_callback)

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

        self.moveCounter = 0

        self.distance_x = 0.0
        self.distance_y = 0.0
        self.prev_linear_velocity_x = None
        self.prev_linear_velocity_y = None

        self.test_var = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.counter = 0
        self.prev_distance_forward = None
        self.prev_distance_backward = None
        self.prev_distance_right = None
        self.prev_distance_left = None
        self.roll_x = 0.0
        self.pitch_y =0.0
        self.yaw_z =0.0

        self.dt = 0.01  # Assuming a constant time step
        self.vx = 0.0   # Initial velocity in x direction
        self.vy = 0.0   # Initial velocity in y direction
        self.vz = 0.0   # Initial velocity in z direction
        self.px = 0.0    # Initial position in x direction
        self.py = 0.0    # Initial position in y direction
        self.pz = 0.0    # Initial position in z direction

        self.pid = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid.output_limits = (-0.25, 0.25)

    def timeSleep(self, duration, dir="f"):
        with timer() as t:
            while t.elapse < duration:
                rospy.sleep(0.1)
                if dir == "f":
                    if self.obstacleAverage <1:
                        return False
                elif dir == "b":
                    if self.backObstacleAverage <1:
                        return False
    
    def state_callback(self, msg):
        self.currentTrot = msg.data
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

    def odom_callback(self, msg):
        # Extract orientation
        self.x_quaternion = msg.orientation.x
        self.y_quaternion = msg.orientation.y
        self.z_quaternion = msg.orientation.z
        self.w_quaternion = msg.orientation.w
        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x_quaternion, self.y_quaternion, self.z_quaternion, self.w_quaternion)

        # Extract angular velocity
        self.angular_velocity_x = msg.angular_velocity.x
        self.angular_velocity_y = msg.angular_velocity.y
        self.angular_velocity_z = msg.angular_velocity.z

        # Extract linear acceleration
        self.linear_acceleration_x = msg.linear_acceleration.x
        self.linear_acceleration_y = msg.linear_acceleration.y
        self.linear_acceleration_z = msg.linear_acceleration.z

    def slam_callback(self, msg):
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
        self.x_quaternion = msg.pose.orientation.x
        self.y_quaternion = msg.pose.orientation.y
        self.z_quaternion = msg.pose.orientation.z
        self.w_quaternion = msg.pose.orientation.w
        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(self.x_quaternion, self.y_quaternion, self.z_quaternion, self.w_quaternion)
        # self.yaw_z += 1.57

    @contextmanager
    def timer(self):
        start = time.time()
        class Timer:
            @property
            def elapse(self):
                return time.time() - start
        yield Timer()

    def move_forward_for_duration(self, duration):
        msg = self.current_joy_message
        msg.axes[1] = 0.5  # Assuming positive value moves forward
        self.current_joy_message = msg
        self.publish_current_command()

        initial_yaw = self.yaw_z  # Capture the initial yaw value
        self.pid.setpoint = initial_yaw  # Set the PID controller setpoint to the initial yaw

        with self.timer() as t:
            while t.elapse < duration:
                msg = self.current_joy_message

                # Calculate the correction factor using the PID controller
                correction = self.pid(self.yaw_z - initial_yaw)
                
                print("help")
                print("SLAM Yaw: ", self.yaw_z)
                print("Correction: ", correction)
                msg.axes[3] = correction  # Apply the correction to the robot's direction

                self.current_joy_message = msg
                self.publish_current_command()

                rospy.sleep(0.05)
                if self.obstacleAverage < 1:
                    return False

        msg.axes[1] = 0.0  # Stop movement after duration
        msg.axes[3] = 0.0
        self.current_joy_message = msg
        self.publish_current_command()

    def mag(self, x, y):
        return sqrt(pow(x,2) + pow(y,2))

    def move_straight_for_distance(self, distance):
        print("Distance")
        print(distance)

        msg = self.current_joy_message

        reference_x = self.px
        reference_y = self.py
        
        current_x = 0.0
        current_y = 0.0
        current_mag = 0.0

        msg.axes[1] = 0.5  # Assuming positive value moves forward
        # msg.axes[3] = 0.1 
        self.current_joy_message = msg
        self.publish_current_command()

        initial_yaw = self.yaw_z  # Capture the initial yaw value
        self.pid.setpoint = initial_yaw  # Set the PID controller setpoint to the initial yaw


        while(current_mag <= distance):
            current_x = self.px - reference_x
            current_y = self.py - reference_y
            current_mag = self.mag(current_x,current_y)
            print("PX: ", self.px)
            print("PY", self.py)
            print("Current mag", current_mag)

            # Calculate the correction factor using the PID controller
            correction = self.pid(self.yaw_z - initial_yaw)
            
            print("help")
            print("SLAM Yaw: ", self.yaw_z)
            print("Correction: ", correction)
            msg.axes[3] = correction  # Apply the correction to the robot's direction

            self.current_joy_message = msg
            self.publish_current_command()
            rospy.sleep(0.05)
            # if self.timeSleep(0.1) == False:
            #     breakself.yaw_z
        
        msg.axes[1] = 0.0  # Stop movement
        msg.axes[3] = 0.0 
        self.current_joy_message = msg
        self.publish_current_command()

    def enable(self):
        msg = self.current_joy_message
        msg.buttons[5] = 1  # Set button corresponding to '1' key to 1
        self.current_joy_message = msg
        self.publish_current_command()
        msg.buttons[5] = 0
        self.publish_current_command()

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
        auto_mover.timeSleep(7)

        while not rospy.is_shutdown():
            if auto_mover.currentTrot == 0 and auto_mover.disabled == False:
                auto_mover.enable()
                auto_mover.timeSleep(1)
            if auto_mover.disabled == False and auto_mover.currentTrot != 0:
                print("this is a test")
                # auto_mover.move_forward_for_duration(15)
                auto_mover.move_straight_for_distance(2.9)
                while auto_mover.currentTrot != 0:
                    auto_mover.enable()
                    auto_mover.disabled = True
                    auto_mover.timeSleep(0.5)
                    rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard node interrupted.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard node stopped by user.")
    finally:
        rospy.loginfo("Exiting...")

if __name__ == "__main__":
    main()

