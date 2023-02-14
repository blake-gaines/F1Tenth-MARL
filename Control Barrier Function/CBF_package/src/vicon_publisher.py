#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from acc_package.msg import acc_msg
import numpy as np
import os

class ViconPublisher:
	def __init__(self):
		self.ego_y_old = 0  
		self.ego_y_new = 0
		self.ego_y = 0
		self.ego_vel = 0
		self.ego_window = np.zeros(10)
		self.frontcar_y_old = 0
		self.frontcar_y_new = 0
		self.frontcar_vel = 0
		self.frontcar_y = 0
		self.frontcar_window = np.zeros(10)
		self.frontcar_y_window = np.zeros(10)
		self.ego_y_window = np.zeros(10)

		self.path = "/home/turtlebot/yash_shubham_ws/logs"
		self.ego_file = open(os.path.join(self.path,"ego_stats.csv"),"w")
		self.frontcar_file = open(os.path.join(self.path,"frontcar_stats.csv"),"w")
		self.distance_file = open(os.path.join(self.path,"distance_stats.csv"),"w")
		self.acc_file = open(os.path.join(self.path,"acc.csv"),"w")
		self.command_vel_file = open(os.path.join(self.path,"command_vel.csv"),"w")
		
		self.ego_file.write("time, pos, vel, fil_pos, fil_vel\n")
		self.frontcar_file.write("time, pos, vel, fil_pos, fil_vel\n")
		self.distance_file.write("time, z, fil_z\n")
		self.acc_file.write("time, acc\n")
		self.command_vel_file.write("time, vel\n")

			
	def ego_callback(self, msg):

		'''
		Applying median filter on a window of size 10 to refine the noisy sensor values. This function applies median filter on position and velocity values of ego vehicle.
		'''

		self.ego_y_new = float(msg.transform.translation.y)
		self.ego_window = np.delete(self.ego_window, 0)
		self.ego_window = np.append(self.ego_window, (self.ego_y_new - self.ego_y_old)/0.01)
		self.ego_y_old = self.ego_y_new
		
		self.ego_y_window = np.delete(self.ego_y_window, 0)
		self.ego_y_window = np.append(self.ego_y_window, self.ego_y_new)
		
		self.ego_y = np.percentile(self.ego_y_window, 50)
		self.ego_vel = np.percentile(self.ego_window, 50)
		
		self.ego_file.write("{}.{},{},{},{},{}\n".format(msg.header.stamp.secs, msg.header.stamp.nsecs, self.ego_y_new, 
													 (self.ego_y_new - self.ego_y_old)/0.01, self.ego_y, self.ego_vel))

	def frontcar_callback(self, msg):

		'''
		Applying median filter on a window of size 10 to refine the noisy sensor values. This function applies median filter on position and velocity values of front vehicle.
		'''

		self.frontcar_y_new = float(msg.transform.translation.y)
		self.frontcar_window = np.delete(self.frontcar_window, 0)
		self.frontcar_window = np.append(self.frontcar_window, (self.frontcar_y_new - self.frontcar_y_old)/0.01)
		self.frontcar_y_old = self.frontcar_y_new

		self.frontcar_y_window = np.delete(self.frontcar_y_window, 0)
		self.frontcar_y_window = np.append(self.frontcar_y_window, self.frontcar_y_new)
		
		self.frontcar_y = np.percentile(self.frontcar_y_window, 50)
		self.frontcar_vel = np.percentile(self.frontcar_window, 50)

		self.frontcar_file.write("{}.{},{},{},{},{}\n".format(msg.header.stamp.secs, msg.header.stamp.nsecs, self.frontcar_y_new, 
													 (self.frontcar_y_new - self.frontcar_y_old)/0.01, self.frontcar_y, self.frontcar_vel))

	def acceleration_callback(self, msg):
		t = rospy.Time.now()
		self.acc_file.write("{}.{},{}\n".format(t.secs, t.nsecs, msg.input_type))

	def command_vel_callback(self, msg):
		t = rospy.Time.now()
		self.command_vel_file.write("{}.{},{}\n".format(t.secs, t.nsecs, msg.linear.x))


if __name__ == "__main__":
	pub = rospy.Publisher('acc/feedback', acc_msg, queue_size=1)
	obj = ViconPublisher()
	rospy.Subscriber('/vicon/turtlebot_traj_track/turtlebot_traj_track', TransformStamped, obj.ego_callback) # Subscrber to obtain ego vehicle position 
	rospy.Subscriber('/vicon/frontcar_traj_track/frontcar_traj_track', TransformStamped, obj.frontcar_callback) # Subscrber to obtain front vehicle position
	rospy.Subscriber('/acc/input', acc_msg, obj.acceleration_callback) # Subscribe to controller acceleration values
	rospy.Subscriber('/mobile_base/commands/velocity', Twist, obj.command_vel_callback) #Subscriber to previous state velocities of ego vehicle

	rospy.init_node('vicon_pub')
	# rospy.spin()
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		fdbk = acc_msg()
		z = obj.frontcar_y - obj.ego_y
		t = rospy.Time.now()
		obj.distance_file.write("{}.{},{},{}\n".format(t.secs, t.nsecs, obj.frontcar_y_new - obj.ego_y_new, z))
		fdbk.fdbk_type = [obj.ego_vel, z, obj.ego_vel, obj.frontcar_vel]
		pub.publish(fdbk)
		rate.sleep()