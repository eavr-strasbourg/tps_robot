#!/usr/bin/env python

'''
Created on: 4 Oct 2013
Author: Olivier Kermorgant

Setpoint generator for servo4Point simulator
ROS example

Subscribes none

Publishes: 
- robot position setpoints on /setpoint

'''
 
 # ROS stuff and multithreading
import roslib; roslib.load_manifest('tpRobots')
import rospy,time,threading
from std_msgs.msg import Float32MultiArray

rospy.init_node('SetpointGenerator')

# setpoint-level sampling time
Te = .1
# setpoint duration
Ts = 3
# setpoint value 
v = .1

# Ts as a parameter
if rospy.has_param('/Ts') == False:
	rospy.set_param('/Ts', Ts)

# Publisher for setpoint
setpointPub = rospy.Publisher('/setpoint', Float32MultiArray)
setpointMsg = Float32MultiArray()
setpointMsg.data = [0,0]

it = 0
while not rospy.is_shutdown():
	it+=1
	
	# get Ts
	Ts = rospy.get_param('Ts')
	count = int(Ts/Te)
  
	if it % count == 0:
		if it == count:
			setpointMsg.data = [-v,-v]
		elif it == 2*count:
			setpointMsg.data = [-v,v]
		elif it == 3*count:
			setpointMsg.data = [v,v]
		else:
			setpointMsg.data = [v,-v]
			it = 1
                print 'Changing to :', setpointMsg.data
	setpointPub.publish(setpointMsg)
	time.sleep(Te)
    
