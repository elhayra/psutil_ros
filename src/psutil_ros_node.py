#!/usr/bin/env python

import rospy
import psutil
from psutil_ros.msg import Cpu
from psutil_ros.msg import VirtualMemory
from psutil_ros.msg import Network
from psutil_ros.msg import Disk


if __name__ == '__main__':
    
    rospy.init_node('psutil_ros_node', anonymous=True)

    # parameters
    loop_rate = rospy.get_param('~loop_rate', 1)
    publish_cpu = rospy.get_param('~publish_cpu', True)
    publish_memo = rospy.get_param('~publish_memo', True)
    publish_disk = rospy.get_param('~publish_disk', True)
    publish_network = rospy.get_param('~publish_network', True)
    
    # publishers 
    cpu_pub = rospy.Publisher('psutil_ros/cpu', Cpu, queue_size=10)
    mem_pub = rospy.Publisher('psutil_ros/virtual_memory', VirtualMemory, queue_size=10)
    disk_pub = rospy.Publisher('psutil_ros/disk', Disk, queue_size=10)
    network_pub = rospy.Publisher('psutil_ros/network', Network, queue_size=10)
    
    rate = rospy.Rate(loop_rate)
    is_first = True
    
    while not rospy.is_shutdown():
	
	# messages 
	cpu_msg = Cpu()
	memo_msg = VirtualMemory()
	disk_msg = Disk()
        network_msg = Network()
	
	#********* CPU *********
	cpu_msg.usage = psutil.cpu_percent(interval=0)
	cpu_msg.physical_cores = psutil.cpu_count(logical=False)
	cpu_msg.cores = psutil.cpu_count()

	# read cpu data only after first cycle, because we are using interval=0, and psutil'''
	# documentation says in this case, the first read should be ignored'''
	if not is_first and publish_cpu: 
            cpu_pub.publish(cpu_msg)
            
            
            
        #**** Virtual Memo *****
        if publish_memo:
            mem_pub.publish(memo_msg)
        
        #********* Disk ********  
        if publish_disk:
            disk_pub.publish(disk_msg)
            
        #******* Network *******   
        if publish_network:
            network_pub.publish(network_msg)
        
        rate.sleep()
	is_first = False
