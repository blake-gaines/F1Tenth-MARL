#!/usr/bin/env python
import numpy as np
import math
from cvxopt import matrix, solvers
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
from acc_package.msg import acc_msg
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import rospy
import pickle

class Solver:
	def __init__(self):
		self.scale = 2.5 # Scale the ego vehicle trajectory waypoints depending on the scale of input
		self.Aclf_stor = [] 
		self.Bclf_stor = []
		self.B_stor = []
		self.const_mode = 1 
		self.dt = 0.01
		self.tf = 48 # Maximum time required for the ego vehicle to complete the trajectory
		self.maxsteps = self.tf/self.dt
		self.step = 0
		self.old_vel = 0 #Previous state ego vehicle linear velocity
		self.old_omega = 0 #Previous state ego vehicle angular velocity

		self.X = np.zeros((4, self.maxsteps))
		self.U_plot = np.zeros((2, self.maxsteps))
		# self.points = np.array([[-250, -50], [-215, -5], [-175, 40], [-115, 20], [-70, -20], [-10, -10]])
		# self.t = np.array([0, 6, 12, 18, 24, 30])
		self.points = np.array([[-250, 0], [0, 250], [250, 0], [0, -250], [-250, 0]])*1.0/self.scale #Waypoints of ego vehicle in cm
		self.t = 2*np.array([0, 6, 12, 18, 24]) #Time elapsed when the ego vehicle reaches the waypoints

		self.points_obs = 100*np.array([[-270, 40], [-215, 40], [-175, 40], [-138, 35], [-110, 15], [-90, -10], [-70, -35], [-50, 10]]) #Waypoints of obstacle in cm
		self.t_obs = np.array([0, 6, 12, 18, 24, 30, 36, 42]) #Time elapsed when the obstacle reaches the waypoints

		self.trajectory = self.getTrajectory(self.points, self.t)
		self.trajectory_obs = self.getTrajectory(self.points_obs, self.t_obs)

		self.vmin = 0.05*100  #
		self.vmax = 1.5*100
		self.wmax = 0.671

		self.pubvel = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)


	def getTrajectory(self, points, t, V_init=0, V_final=0, dt=0.01, Tf=48):
		'''
		This function returns the trajectory coordinates for the x and y axes along with the reference values for the heading theta, linear velocity and angular velocity
		'''

		x = points[:,0]
		y = points[:,1]
		N = int(Tf*1.0/dt)
		tq = np.linspace(0, Tf, N)
		
		xpol = UnivariateSpline(t, x, k=3, s=0) #Creates a spline for the trajectory
		xdpol = xpol.derivative(n=1)
		xddpol = xpol.derivative(n=2)

		xq = xpol(tq)
		xdq = xdpol(tq)
		xddq = xddpol(tq)

		ypol = UnivariateSpline(t, y, k=3, s=0)
		ydpol = ypol.derivative(n=1)
		yddpol = ypol.derivative(n=2)

		yq = ypol(tq)
		ydq = ydpol(tq)
		yddq = yddpol(tq)
		K = np.array([(xdq[i]*yddq[i] - xddq[i]*ydq[i])/math.pow(xdq[i]**2 + ydq[i]**2, 1.5) for i in range(xdq.shape[0])])
		# K = np.zeros(xq.shape[0])

		wref = K*xdq
		wref[0] = 0

		thetaref = np.array([math.atan2(ydq[i], xdq[i]) for i in range(xdq.shape[0])])
		# vref = xdq*np.array([math.cos(thetaref[i]) for i in range(thetaref.shape[0])]) + ydq*np.array([math.sin(thetaref[i]) for i in range(thetaref.shape[0])])
		vref = np.array([math.sqrt(xdq[i]*xdq[i] + ydq[i]*ydq[i]) for i in range(xdq.shape[0])])

		xq = xq.reshape((N,1))
		yq = yq.reshape((N,1))
		thetaref = thetaref.reshape((N,1))
		vref = vref.reshape((N,1))
		wref = wref.reshape((N,1))
		traj = np.hstack((xq, yq, thetaref, vref, wref)).T
		return (traj)

	def error(self, desired, actual):

		'''
		Function to measure angular errors since angular values are cyclic. 
		'''
		diff = desired - actual
		if (abs(diff) <= math.pi):
			return diff
		elif diff < -math.pi:
			return diff + 2*math.pi
		else:
			return diff - 2*math.pi

	def controller(self, value):
		x, y, theta, z, xd, yd, thetad, vd, wd, theta0, v0 = value
		vmin = self.vmin
		vmax = self.vmax
		wmax = self.wmax
		m = 1
		k1 = 1
		fsilon = 1
		gamma = 0.001
		# gamma = 0.5

		#Rotation Matrix
		R = np.array([[math.cos(theta), math.sin(theta), 0],
			[-math.sin(theta), math.cos(theta), 0], 
			[0, 0, 1]])


		#Errors in vehicle frame given be R.(errors in reference frame)
		X_e = R.dot(np.array([[xd-x],[yd-y],[self.error(thetad, theta)]]))
		# X_e = R.dot(np.array([[xd-x],[yd-y],[thetad-theta]]))
		x0 = X_e[2,0]
		x1 = X_e[1,0]
		x2 = -X_e[0,0]


		# If error in x axis and theta approaches 0, error in y_axis becomes uncontrollable. x0 and pi introduced to counter the same.
		pi1 = math.sqrt(x1**2 + x2**2 +1)

		x0_ = m*x0 + x1/pi1


		pi2 = math.sqrt(x0_**2 + 1)


		'''
		Non-Linear Affine control system of the form x_dot = f(x) + g(x) *u
		'''
		f = np.vstack((x2*wd/pi1 + (1+x2**2)*vd*math.sin(x0)/pi1**3,
					  x2*wd + vd*math.sin(x0),
					  -wd*x1))

		g = np.array([[m-(x2/pi1), -x1*x2/pi1**3],
					  [-x2, 0],
					  [x1, 1]])

		V = pi2 +  k1*pi1 -(1+k1) # Lyapunov Candidate 

		V_dot = np.array([(x0_/pi2), (k1*x1/pi1), (k1*x2/pi1)]).reshape((1,3))

		si0 = V_dot.dot(f) + fsilon*V  # Control Lyapunov Constraints
		si1 = V_dot.dot(g)

		self.Aclf_stor.append(si1.T)
		self.Bclf_stor.append(si0)

		h = z-10  # 10 is the safety distance. 
		B = -math.log(h/(1+h))  # Barrier Function

		self.B_stor.append(B)

		LfB =  ((-v0 + vd*math.cos(x0)))/(h*(h+1))  # Control Barrier Constraints 
		LgB = np.array([[0, 1/(10*h*(h+1))]])

		y1 = math.sqrt(pi2-1)
		y2 = math.sqrt(k1*pi1 - k1)
		b1 = (m - x2/pi1)/(2*y1*pi2)
		c1 = (-x1*x2/pi1**3)/(2*y1*pi2)
		a1 = (x2*wd/pi1 + (1+x2**2)*vd*math.sin(x0)/pi1**3)/(2*y1*pi2)
		a2 = k1*x1*vd*math.sin(x0)/(2*y2*pi1)
		b2 = 0
		c2 = 1
		psc = math.exp(4)  # Penalty for softness in Lyapunov Constraints

		Aclf = np.hstack((si1, [[-1]]))
		bclf = -si0

		Acbf = np.hstack((LgB, [[0]]))
		bcbf = -LfB + (gamma/B)

		if self.const_mode == 0:
			A = matrix(np.vstack((Aclf, Acbf)))
			b = matrix(np.vstack((bclf, bcbf)))

		elif self.const_mode == 1:
			wu = wd + wmax 
			wl = wd - wmax
			vu = vmax - vd*math.cos(x0) 
			vl = vmin - vd*math.cos(x0)

			Acc = np.array([[1,0,0],[-1,0,0],[0,1,0]])
			bcc = np.vstack((wu, -wl, vu))

			A = matrix(np.vstack((Aclf, Acbf, Acc)))
			b = matrix(np.vstack((bclf, bcbf, bcc)))

		else:
			print('Not a valid mode')

		H = 2.0*matrix([[(b1**2+b2**2), (b1*c1+b2*c2), 0],
						[(b1*c1+b2*c2), (c1**2+c2**2), 0],
						[0, 0, psc]])

		f = 2.0*matrix(np.array([[a1*b1 + a2*b2], [a1*c1 + a2*c2], [0]]))
		solvers.options['show_progress'] = False
		U1 = solvers.qp(H,f,A,b)   # Quadratic Programming optimizer

		wc = wd - U1['x'][0]
		vc = U1['x'][1] + vd*math.cos(x0)

		# vc = u0
		# wc = u1

		U = np.array([vc, wc])
		return(U)

	def callback(self, msg):
		if self.step > self.maxsteps-1:
			return
		i = self.step
		self.X[0,i] = msg.fdbk_type[0]*100.0
		self.X[1,i] = msg.fdbk_type[1]*100.0
		self.X[2,i] = msg.fdbk_type[2]
		self.trajectory_obs[0,i] = msg.fdbk_type[3]*100.0
		self.trajectory_obs[1,i] = msg.fdbk_type[4]*100.0
		self.trajectory_obs[2,i] = msg.fdbk_type[5]
		self.trajectory_obs[3,i] = 0
		# X[:,0] = [-315, -10, 2.75, 0]
		# X[:,0] = [-250/scale, 0.1, 1.57, 0]
		self.X[3,i] = math.sqrt((self.X[0,i] - self.trajectory_obs[0,i])**2 + (self.X[1,i] - self.trajectory_obs[1,i])**2)
		U = self.controller(np.hstack((self.X[:4,i], self.trajectory[:,i], self.trajectory_obs[2:4,i])))
		U[0] = U[0]/100.0

		self.U_plot[:,i] = U

		if (U[0] - self.old_vel) > 0.002:
			U[0] = self.old_vel + 0.002
		elif  (U[0] - self.old_vel) < -0.002 and U[0] < 0:
			U[0] = self.old_vel - 0.002
		self.old_vel = U[0]

		# if (U[1] - self.old_omega) > 0.002:
		# 	U[1] = self.old_omega + 0.002
		# elif  (U[1] - self.old_omega) < -0.002:
		# 	U[1] = self.old_omega - 0.002
		# self.old_omega = U[1]

		if i > 10:
			self.pubvel.publish(Twist(Vector3(U[0],0,0),
	                   Vector3(0,0,U[1])))
		self.step += 1

if __name__ == "__main__":

	obj = Solver()
	rospy.init_node('2d_track')
	rate = rospy.Rate(100)
	rospy.Subscriber('/twoD/input', acc_msg, obj.callback)
	
	while obj.step < obj.maxsteps-1:
		continue
		
	data = {'states' : obj.X, 'desired' : obj.trajectory, 'commands' : obj.U_plot}
	pickle.dump(data, open('/home/turtlebot/yash_shubham_ws/data.pickle','wb'))