#!/usr/bin/env python
'''
create acc_simulation node to simulate state feedback for
acc, which publish to the acc/feedback topic 
fdbk_type is data type where data.fdbk_type = [v, z, u_old]
v -  velocity of ego vehicle
z - distance between ego and front vehicle
u_old - running average of input
'''


import rospy
import argparse
from acc_package.msg import acc_msg 
# from acc_package.msg import acc_msg.fdbk_type as fdbk_type

import numpy as np
import math
import matplotlib.pyplot as plt
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
'''
system constant get the value from ROSPARAM
'''
vd = rospy.get_param("/acc/vd")         # Desired velocity of ego vehicle (m/s)
v = rospy.get_param("/acc/v")           # Initial velocity of ego vehicle (m/s)
v0 = rospy.get_param("/acc/v0")         # Front vehicle velocity (m/s)
z = rospy.get_param("/acc/z")           # Initial Distance between front vehicle and ego vehicle (m)
dt = rospy.get_param("/acc/dt")         # Value of time step (s)

m = rospy.get_param("/acc/m")           # mass of ego vehicle (kg)
ca = rospy.get_param("/acc/ca")         # max acceleration ca*g
cd = rospy.get_param("/acc/cd")         # max deacceleration -cd*g
g = rospy.get_param("/acc/g")           # gravitational acceleration

fsilon = rospy.get_param("/acc/fsilon") # coeff to Exponential CLF
gamma = rospy.get_param("/acc/gamma")   # Coeff to exponential CBF
x0 = rospy.get_param("/acc/x0")         # Stopping distance from front vehicle
    
f0 = rospy.get_param("/acc/f0")         # Coeff for aerodynamic drag
f1 = rospy.get_param("/acc/f1")
f2 = rospy.get_param("/acc/f2")

A = rospy.get_param("/acc/A")           # Front vehicle velocity amplitude
f = rospy.get_param("/acc/f")           # Front vehicle velocity frequency
profile = rospy.get_param("/acc/profile")

pub = rospy.Publisher('acc/feedback', acc_msg, queue_size=1)
pub_z = rospy.Publisher('acc/z', acc_msg, queue_size=1)
pub_v = rospy.Publisher('acc/v', acc_msg, queue_size=1)
pub_v0 = rospy.Publisher('acc/v0', acc_msg, queue_size=1)


class State:
    def __init__(self):
        """
        Vehicle state
        v - velocity
        z - distance between ego vehicle and front vehicle
        a - acceleration (input)
        u_old - old input to calculate running average for jerk
        """
        self.v = v
        self.z = z
        self.a = 0.0
        self.u_old = 3*[0.0]
        self.t = 0
        self.v0 = v0
        self.profile = profile

    def input_callback(self, input_msg):
        '''
        input callback to get input from acc/input node
        and update the acc state
        data type is input_type where data.input_type = u
        '''
        u = input_msg.input_type

        if u <= ca*g:          #upper limit constraints on input         
            self.a = u
        else:
            self.a = ca*g
        if u >= -cd*g:
           self.a = u
        else:
           self.a = -cd*g
        self.update_time()
        self.get_velocity()
        self.update()        

    def get_velocity(self):
        if self.profile == "sinusoidal":
            self.v0 = A+A*math.sin(2*math.pi*f*self.t)
        elif self.profile == "linear":
            self.v0 = v0

    def update_time(self):
        self.t += dt
   
    def update(self):
        """
        updatation for state
        """
        self.z += dt*(self.v0 - self.v)
        self.v += dt*self.a;
    

def simulation():
    """
    simulation of ACC
    initialization of state:
    default intial velocity = 20m/s and
    initial distance between ego and front vehicle is 100m
    """
    state = State() 

    '''
    subscribe to acc/input topic to get input
    '''
    rospy.Subscriber('acc/input', acc_msg, state.input_callback)
    rospy.init_node('acc_simulation')   # initialize node for simulation
    rate = rospy.Rate(100)              # node frequency 100hz
    while not rospy.is_shutdown():

        """
        running average calculation 
        """
        run_avg = state.u_old[2] + 0.3*(state.u_old[2] - state.u_old[0]) 

        fdbk = acc_msg()
        fdbk.fdbk_type = [state.v, state.z, run_avg]
        pub.publish(fdbk)       # publish feedback
        
        v0_msg = acc_msg()
        v0_msg.input_type = state.v0
        pub_v0.publish(v0_msg)

        v_msg = acc_msg()
        v_msg.input_type = state.v
        pub_v.publish(v_msg)
        
        z_msg = acc_msg()
        z_msg.input_type = state.z
        pub_z.publish(z_msg)

        rate.sleep()
        

if __name__ == '__main__':
    try:
        simulation()
    except rospy.ROSInterruptException:
        pass
