#!/usr/bin/env python

import rospy
import psutil
from psutil_ros.msg import Cpu


if __name__ == '__main__':
    
    loop_rate = 1
    
    rospy.init_node('psutil_ros_node', anonymous=True)
    
    ''' publishers '''
    cpu_pub = rospy.Publisher('psutil_ros/cpu', Cpu, queue_size=10)
    network_pub = rospy.Publisher('psutil_ros/network', Network, queue_size=10)
    disk_pub = rospy.Publisher('psutil_ros/disk', Disk, queue_size=10)
    mem_pub = rospy.Publisher('psutil_ros/virtual_memory', VirtualMemory, queue_size=10)
    
    rate = rospy.Rate(loop_rate)
    is_first = True
    while not rospy.is_shutdown():
	'''read cpu data only after first cycle, because we are using interval=0, and psutil'''
	'''documentation says in this case, the first read should be ignored'''
	if not is_first: 
		msg = Cpu()
        	msg.usage = psutil.cpu_percent(interval=0)
        	cpu_pub.publish(msg)
        rate.sleep()
	is_first = False
