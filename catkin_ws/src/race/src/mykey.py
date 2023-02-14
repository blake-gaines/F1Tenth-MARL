#!/usr/bin/env python

import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
import sys
from std_msgs.msg import Float64
import keyboard

# arbitrarily initialized. 25 is not a special value. This code can accept input desired velocity from the user.
angle_min_rel     = -100.0              # max left command
angle_max_rel     = 100.0               # max right command
angle_min_abs     = 0.0                 # max left VESC servo
angle_max_abs     = 1.0                 # max right VESC servo

speed_min_rel     = -100.0              # min speed command
speed_max_rel     = 100.0               # max speed command


rospy.init_node('keyboard', anonymous=True)
angle_pub         = rospy.Publisher('/commands/servo/position', Float64, queue_size = 1)
speed_pub         = rospy.Publisher('/commands/motor/duty_cycle', Float64, queue_size = 1)
# setup a publisher to publish to the /car_x/offboard/command topic for your racecar.

def output_angle_mixer(rel_angle):
    output_angle = (rel_angle - angle_min_rel)/(angle_max_rel - angle_min_rel)
    output_angle = output_angle * (angle_max_abs - angle_min_abs)
    return output_angle 

def output_speed_mixer(rel_speed):
    global duty_cycle
    if rel_speed >= 100.0: rel_speed = 100.0
    if rel_speed <= -100.0: rel_speed = -100.0
    duty_cycle = (rel_speed - 0)/(speed_max_rel - 0)
    return duty_cycle



while True:
    if keyboard.is_pressed('w'):
        vel_input = 4
        print("w is pressed", vel_input)
    if keyboard.is_pressed('s'):
        vel_input = 0
        print("s is pressed", vel_input)
    if keyboard.is_pressed('a'):
        angle = -30
        print("a is pressed", angle)
    if keyboard.is_pressed('d'):
        angle = 30
        print("d is pressed", angle)
    if keyboard.is_pressed('x'):
        angle = 0
        print("x is pressed", angle)
    if keyboard.is_pressed('q'):
        quit()  
    angle_req = Float64()
    speed_req = Float64()
    angle_req.data = output_angle_mixer(angle)
    speed_req.data = output_speed_mixer(vel_input)
    angle_pub.publish(angle_req)
    speed_pub.publish(speed_req)
    

