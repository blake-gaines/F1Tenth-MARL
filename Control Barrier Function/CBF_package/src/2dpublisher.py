#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from acc_package.msg import acc_msg
import numpy as np
import os
import math
class ViconPublisher:
	def __init__(self):
		self.window_size = 10
		self.ego_x = 0
		self.ego_y = 0
		self.ego_orientation = 0
		self.ego_median_x = 0
		self.ego_median_y = 0
		self.ego_median_orientation = 0

		self.ego_x_window = np.zeros(self.window_size)
		self.ego_y_window = np.zeros(self.window_size)
		self.ego_orientation_window = np.zeros(self.window_size)

		self.frontcar_x = 0
		self.frontcar_y = 0
		self.frontcar_orientation = 0
		self.frontcar_median_x = 0
		self.frontcar_median_y = 0
		self.frontcar_median_orientation = 0

		self.frontcar_x_window = np.zeros(self.window_size)
		self.frontcar_y_window = np.zeros(self.window_size)
		self.frontcar_orientation_window = np.zeros(self.window_size)

		# self.frontcar_y_old = 0
		# self.frontcar_y_new = 0
		# self.frontcar_vel = 0
		# self.frontcar_y = 0
		# self.frontcar_window = np.zeros(10)
		# self.frontcar_y_window = np.zeros(10)
		# self.ego_y_window = np.zeros(10)

		# self.path = "/home/turtlebot/yash_shubham_ws/logs"
		# self.ego_file = open(os.path.join(self.path,"ego_stats.csv"),"w")
		# self.frontcar_file = open(os.path.join(self.path,"frontcar_stats.csv"),"w")
		# self.distance_file = open(os.path.join(self.path,"distance_stats.csv"),"w")
		# self.acc_file = open(os.path.join(self.path,"acc.csv"),"w")
		# self.command_vel_file = open(os.path.join(self.path,"command_vel.csv"),"w")
		
		# self.ego_file.write("time, pos, vel, fil_pos, fil_vel\n")
		# self.frontcar_file.write("time, pos, vel, fil_pos, fil_vel\n")
		# self.distance_file.write("time, z, fil_z\n")
		# self.acc_file.write("time, acc\n")
		# self.command_vel_file.write("time, vel\n")

			
	def ego_callback(self, msg):
		self.ego_x = float(msg.transform.translation.x)
		self.ego_y = float(msg.transform.translation.y)
		orientation = euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
		self.ego_orientation = np.array(list(orientation))[2]

		self.ego_x_window = np.delete(self.ego_x_window, 0)
		self.ego_x_window = np.append(self.ego_x_window, self.ego_x)

		self.ego_y_window = np.delete(self.ego_y_window, 0)
		self.ego_y_window = np.append(self.ego_y_window, self.ego_y)

		self.ego_orientation_window = np.delete(self.ego_orientation_window, 0)
		self.ego_orientation_window = np.append(self.ego_orientation_window, self.ego_orientation)
		
		# self.ego_y_window = np.delete(self.ego_y_window, 0)
		# self.ego_y_window = np.append(self.ego_y_window, self.ego_y_new)
		
		self.ego_median_x = np.percentile(self.ego_x_window, 50)
		self.ego_median_y = np.percentile(self.ego_y_window, 50)
		self.ego_median_orientation = np.percentile(self.ego_orientation_window, 50)
		# self.ego_vel = np.percentile(self.ego_window, 50)
		
		# self.ego_file.write("{}.{},{},{},{},{}\n".format(msg.header.stamp.secs, msg.header.stamp.nsecs, self.ego_y_new, 
													 # (self.ego_y_new - self.ego_y_old)/0.01, self.ego_y, self.ego_vel))

	def frontcar_callback(self, msg):
		self.frontcar_x = float(msg.transform.translation.x)
		self.frontcar_y = float(msg.transform.translation.y)
		orientation = euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
		self.frontcar_orientation = np.array(list(orientation))[2]

		self.frontcar_x_window = np.delete(self.frontcar_x_window, 0)
		self.frontcar_x_window = np.append(self.frontcar_x_window, self.frontcar_x)

		self.frontcar_y_window = np.delete(self.frontcar_y_window, 0)
		self.frontcar_y_window = np.append(self.frontcar_y_window, self.frontcar_y)

		self.frontcar_orientation_window = np.delete(self.frontcar_orientation_window, 0)
		self.frontcar_orientation_window = np.append(self.frontcar_orientation_window, self.frontcar_orientation)
		
		# self.frontcar_y_window = np.delete(self.frontcar_y_window, 0)
		# self.frontcar_y_window = np.append(self.frontcar_y_window, self.frontcar_y_new)
		
		self.frontcar_median_x = np.percentile(self.frontcar_x_window, 50)
		self.frontcar_median_y = np.percentile(self.frontcar_y_window, 50)
		self.frontcar_median_orientation = np.percentile(self.frontcar_orientation_window, 50)

		# self.frontcar_file.write("{}.{},{},{},{},{}\n".format(msg.header.stamp.secs, msg.header.stamp.nsecs, self.frontcar_y_new, 
													 # (self.frontcar_y_new - self.frontcar_y_old)/0.01, self.frontcar_y, self.frontcar_vel))

	# def acceleration_callback(self, msg):
	# 	t = rospy.Time.now()
	# 	self.acc_file.write("{}.{},{}\n".format(t.secs, t.nsecs, msg.input_type))

	# def command_vel_callback(self, msg):
	# 	t = rospy.Time.now()
	# 	self.command_vel_file.write("{}.{},{}\n".format(t.secs, t.nsecs, msg.linear.x))


if __name__ == "__main__":
	pub = rospy.Publisher('twoD/input', acc_msg, queue_size=1)
	obj = ViconPublisher()
	rospy.Subscriber('/vicon/turtlebot_traj_track/turtlebot_traj_track', TransformStamped, obj.ego_callback)
	rospy.Subscriber('/vicon/frontcar_traj_track/frontcar_traj_track', TransformStamped, obj.frontcar_callback)
	# rospy.Subscriber('/acc/input', acc_msg, obj.acceleration_callback)
	# rospy.Subscriber('/mobile_base/commands/velocity', Twist, obj.command_vel_callback)

	rospy.init_node('vicon_pub')
	# rospy.spin()
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		fdbk = acc_msg()
		# z = obj.frontcar_y - obj.ego_y
		# t = rospy.Time.now()
		# obj.distance_file.write("{}.{},{},{}\n".format(t.secs, t.nsecs, obj.frontcar_y_new - obj.ego_y_new, z))
		fdbk.fdbk_type = [obj.ego_median_x, obj.ego_median_y, obj.ego_median_orientation, obj.frontcar_median_x, obj.frontcar_median_y, obj.frontcar_median_orientation]
		pub.publish(fdbk)
		rate.sleep()