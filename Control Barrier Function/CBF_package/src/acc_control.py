#!/usr/bin/env python
'''
Calculation for input using control Luapunov function(CLF)
and control barrier function(CBF) for safety aware ACC
create acc_input node and publish input to the acc/input topic 
input_type is data type where data.input_type = u
'''

import rospy
from acc_package.msg import acc_msg 
import argparse	
import sys
from cvxopt import matrix, solvers
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
'''
system constant
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
profile = rospy.get_param("/acc/profile") # Velocity profile can be set to either "linear" or "sinusoidal" by using rosparam set


pub = rospy.Publisher('acc/input', acc_msg, queue_size=1)
pubvel = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)


class Controller:
    def __init__(self):
        self.internal_vel = v
        self.x2 = v
        self.z = z
        self.u = 0
        self.u_old = 1
        self.t = 0
        self.v0 = v0
        self.profile = profile 
 
    def FR(self):
        '''
        aerodynamic drag force for given ego vehicle velocity(x2)
        '''
        x2 = self.x2
        Fr = f0 + f1*x2 + f2*(x2**2)
        return Fr


    def Si_0(self):
        '''
        calculation of si0 for Lyapunov Constraints
        '''
        x2 = self.x2
        Fr = self.FR()
        si0 = -2.0*((x2 - vd)*Fr/m) + fsilon*((x2-vd)**2)
        return si0

    def Si_1(self):
        '''
        calculation of si1 for Lyapunov Constraints
        '''
        x2 = self.x2
        si1 = 2.0*(x2 - vd)/m
        return si1

    def H(self):
        '''
        calculation of h for barrier function
        '''
        x2 = self.x2
        z = self.z
        h = z - 1.8*x2 - x0 -(0.5*((self.v0-x2)**2))/(cd*g)
        return h


    def B(self):
        '''
        calculation of barrier function
        '''
        x2 = self.x2
        z = self.z
        h = self.H()
        b = -math.log(h/(1.0+h))
        return b


    def LFB (self):
        '''
        calculation of LFB for CBF Constraints
        '''
        x2 = self.x2
        z = self.z
        Fr = self.FR()
        h = self.H()
        Lfb = -(-Fr*(self.v0-x2)/(cd*g*m) + 1.8*Fr/m + self.v0 - x2)/(h*(1.0+h))
        return Lfb


    def LGB(self):
        '''
        calculation of LGB for CBF Constraints
        '''
        x2 = self.x2
        z = self.z
        h = self.H()
        Lgb = (1.8 - (self.v0-x2)/(cd*g))/(m*h*(1.0+h))
        return Lgb



    def PSC(self):
        '''
        penality for softness in Lyapunov constraints
        '''
        x2 = self.x2
        z = self.z
        b = self.B()
        psc = math.exp((gamma/(10.0*b))-3)
        return psc


    def fdbk_callback(self,fdbk_msg):
        '''
        feedback callback to get current state
        data type is fdbk_type where
        data.fdbk_type = [x2, z, u_old, v0]
        '''
        fdbk = fdbk_msg.fdbk_type

        self.x2 = fdbk[0]
        self.z = fdbk[1]
        self.u_old = fdbk[2]
        self.v0 = fdbk[3]

    def get_velocity(self):
    	'''
    	Front car velocity profile can also be set to either Linear profile or Sinusoidal profile
    	'''
    	if self.profile == "sinusoidal":
        	self.v0 = A+A*math.sin(2*math.pi*f*self.t)
    	elif self.profile == "linear":
        	self.v0 = v0


    def update_time(self):
        self.t += dt

    def solution(self):
        '''
        solve the input(acceleration) for safety aware ACC using CLF
        and CBL concept
        This function will generate input for speed(x2) and current distance
        between ego vehicle and front vehicle
        '''
        # self.update_time()
        # self.get_velocity()  #Uncomment this to use either 'linear or sinusoidal front vehicle velocity profile. The default front vehicle velocity is obtained through vicon_bridge' ros package.
        b = self.B() # Calculates Barrier Function 
        psc = self.PSC() # Penalty for softness in Lyapunov Constraints
        Fr = self.FR() # aerodynamic drag force for given ego vehicle velocity(x2)

        Aclf = self.Si_1()         # control Lyapunov constraint
        bclf = -self.Si_0()  

        Acbf = self.LGB()       # control barrier constraint
        bcbf = -self.LFB() + (gamma/b)


        '''
        formulation for CVXOPT
            min u'*Q*u + p*u
            subject to 
                G*u <= h
        '''
        Q = 2.0*matrix([[1/m**2, 0.0], [0.0, psc]])
        p = -2.0*matrix([Fr/(m**2), 0.0])
        G = matrix([[Aclf, Acbf] ,[-1.0, 0.0]])
        h = matrix([bclf, bcbf])

        solvers.options['show_progress'] = False
        sol = solvers.qp(Q, p, G, h)

        U = sol['x']
        self.u = U[0]/m


def Solve():
    '''
    initialization of controller 
    default state:
    velocity = 20, distence = 100
    '''
    controller = Controller()

    '''
    subscribe to acc/feedback topic to get current state
    '''
    rospy.Subscriber('acc/feedback', acc_msg, controller.fdbk_callback)

    rospy.init_node('acc_controller')       #initialize acc_controller node
    rate = rospy.Rate(100)                  #node frequency 100hz
    while not rospy.is_shutdown():

        try:            # create exception to get rid of python float round off
            controller.solution()
            u = acc_msg()
            u.input_type = controller.u

        except:
            u = acc_msg()
            u.input_type = controller.u_old

        pub.publish(u) #Publish controller output - acceleration values.
        vel = controller.x2+dt*controller.u  # Computation of velocity given by: v = u+a*t
      

      	# Compute Internal velocity of ego vehicle and to overcome actuator saturation limits
        controller.internal_vel = controller.internal_vel + dt*controller.u
        if abs(controller.internal_vel - vel) > 0.1:
            controller.internal_vel = vel
        print(controller.z)
 

 		# Velocity input to the TurtleBot
        pubvel.publish(Twist(Vector3(controller.internal_vel,0,0),
                   Vector3(0,0,0)))  #


        rate.sleep()


if __name__ == '__main__':
	Solve()
