#!/usr/bin/env python

import rospy
import psutil
from psutil_ros.msg import Cpu
from psutil_ros.msg import VirtualMemory
from psutil_ros.msg import Network
from psutil_ros.msg import Temperatures
from psutil_ros.msg import SensorTemp

#TODO: handle possible exceptions and empty values

if __name__ == '__main__':
    
    rospy.init_node('psutil_ros_node', anonymous=True)

    # parameters
    loop_rate = rospy.get_param('~loop_rate', 1)
    publish_cpu = rospy.get_param('~publish_cpu', True)
    publish_memo = rospy.get_param('~publish_memo', True)
    publish_temps = rospy.get_param('~publish_temps', True)
    publish_network = rospy.get_param('~publish_network', True)
    
    # publishers 
    cpu_pub = rospy.Publisher('psutil_ros/cpu', Cpu, queue_size=10)
    mem_pub = rospy.Publisher('psutil_ros/virtual_memory', VirtualMemory, queue_size=10)
    temps_pub = rospy.Publisher('psutil_ros/temps', Temperatures, queue_size=10)
    network_pub = rospy.Publisher('psutil_ros/network', Network, queue_size=10)
    
    rate = rospy.Rate(loop_rate)
    is_first = True
    
    while not rospy.is_shutdown():
	
	# messages 
	cpu_msg = Cpu()
	memo_msg = VirtualMemory()
	temps_msg = Temperatures()
        network_msg = Network()
	
	#********* CPU *********
	cpu_msg.usage = psutil.cpu_percent(interval=0)
	cpu_msg.physical_cores = psutil.cpu_count(logical=False)
	cpu_msg.cores = psutil.cpu_count()
	cpu_msg.current_freq = psutil.cpu_freq().current
	cpu_msg.min_freq = psutil.cpu_freq().min
        cpu_msg.max_freq = psutil.cpu_freq().max

	# read cpu data only after first cycle, because we are using interval=0, and psutil'''
	# documentation says in this case, the first read should be ignored'''
	if not is_first and publish_cpu: 
            cpu_pub.publish(cpu_msg)
            
        #**** Virtual Memo *****
        if publish_memo:
            vm = psutil.virtual_memory()
            memo_msg.total = vm.total
            memo_msg.available = vm.available
            memo_msg.used = vm.used
            memo_msg.free = vm.free
            memo_msg.percent = vm.percent
            
            mem_pub.publish(memo_msg)
        
        #***** Temperatures ****  
        if publish_temps:
            temps = psutil.sensors_temperatures()

            for key, value in temps.iteritems():
		sensor_temp_msg = SensorTemp()
  		sensor_temp_msg.label = key
		sensor_temp_msg.current = value[0].current
		sensor_temp_msg.high = value[0].high
		sensor_temp_msg.critical = value[0].critical
            	temps_msg.temps.append(sensor_temp_msg)
		
            temps_pub.publish(temps_msg)
            
        #******* Network *******   
        if publish_network:
            net = psutil.net_io_counters()
            network_msg.bytes_sent = net.bytes_sent
            network_msg.bytes_recv = net.bytes_recv
            network_msg.packets_sent = net.packets_sent
            network_msg.packets_recv = net.packets_recv
            network_msg.errin = net.errin
            network_msg.errout = net.errout
            network_msg.dropin = net.dropin
            network_msg.dropout = net.dropout
            
            network_pub.publish(network_msg)
        
        rate.sleep()
	is_first = False
