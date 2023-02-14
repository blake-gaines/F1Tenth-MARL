import rosbag
from acc_package.msg import acc_msg
import matplotlib.pyplot as plt
import numpy as np

bag = rosbag.Bag('data/out.bag')
time = 0
msg_lists = {'/acc/v0':[], '/acc/v':[], '/acc/z':[]}
for topic, msg, t in bag.read_messages(topics=['/acc/v0', '/acc/v', '/acc/z']):
	msg_lists[topic].append(msg.input_type)
	time += 1
	# if time == 10000:
	# 	break
bag.close()
t_v = list(np.array(range(len(msg_lists['/acc/v'])))*0.01)
t_v0 = list(np.array(range(len(msg_lists['/acc/v0'])))*0.01)
t_z = list(np.array(range(len(msg_lists['/acc/z'])))*0.01)

plt.plot(t_z, msg_lists['/acc/z'])
plt.ylabel('Distance between cars (m)')
plt.xlabel('Time (s)')
plt.title('Distance between Cars vs Time')
plt.show()

plt.plot(t_v, msg_lists['/acc/v'], label='Subject Vehicle')
plt.plot(t_v0, msg_lists['/acc/v0'], label='Front Vehicle')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.legend()
plt.title('Velocity of Cars vs Time')
plt.show()