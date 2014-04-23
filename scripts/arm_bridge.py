#!/usr/bin/env python

'''
Created on: 12 Sep 2012
Author: Olivier Kermorgant

Bridge to control arm position and velocity 

Subscribes:
- robot position or velocity command on topic /main_control/command

Publishes:
- robot joint positions commands on topic /joint_control/command (position control)
'''

# modules
# ROS stuff and multithreading
import roslib
import rospy,time,threading
from sensor_msgs.msg import JointState
# math
from pylab import arange, sign
# system
import sys,os

roslib.load_manifest('tps_robot')

verbose = True

# low-level sampling time
T = 1./500

# joint data: read URDF to get joint names and limits (position + velocity)
urdf = rospy.get_param("robot_description").splitlines()
N = 0
jointNames = []
jointMin = []
jointMax = []
jointVelMax = []
inJoint = False
jName = ''
for ui in urdf:
    if inJoint:
        if 'limit' in ui:
            jointNames.append(jName)
            N += 1
            s = ui.split('"')
            for i in xrange(len(s)):
                if 'lower' in s[i]:
                    jointMin.append(float(s[i+1]))
                elif 'upper' in s[i]:
                    jointMax.append(float(s[i+1]))
                elif 'velocity' in s[i]:
                    jointVelMax.append(float(s[i+1]))
            inJoint = False
        elif 'mimic' in ui:
            inJoint = False
    else:
        if 'joint name=' in ui and 'fixed' not in ui:
                jName = ui.split('"')[1]
                inJoint = True

# update max velocity according to sample time T
jointVelMax = [T*v for v in jointVelMax]

print "Initializing bridge with " + str(N) + " joints"

def inJointLimits(q):
    '''
    Returns the position q projected inside the joint limits
    '''
    return [min(max(q[i],jointMin[i]),jointMax[i]) for i in xrange(N)]
    
class State:
	def __init__(self):
	      self.msgPrint = ''
	      self.qSet = [0.]*N
	      self.cmdCount = 0
	      
	def printThread(self):
		'''
		Separate thread that prints stuff
		'''
		msg = ''
		while not rospy.is_shutdown():
			if self.msgPrint != msg:
				msg = self.msgPrint
				if type(msg) == str:	# if msg is a string, print it
					print msg
				else:			# otherwise msg should be a list of values
					print ''.join([str(m) for m in msg])
			time.sleep(T)
			

	def followPosition(self, qDes, cmdCountCur):
		'''
		Follows the joint position setpoint as long as :
		- the corresponding position lies inside the joint limits
		- no other command has been received
		'''
		# build trajectory to go from qSet to qDes with max velocity
		qTraj = [list(arange(self.qSet[i],qDes[i],sign(qDes[i]-self.qSet[i])*jointVelMax[i])[1:]) for i in xrange(N)]
		for i in xrange(N):
			if len(qTraj[i]) == 0:
				qTraj[i].append(qDes[i])
			elif qTraj[i][-1] != qDes[i]:
				qTraj[i].append(qDes[i])
		steps = max([len(t) for t in qTraj])
		for i in xrange(N):
			qTraj[i] += [qDes[i]] * (steps - len(qTraj[i]))

		# follow trajectory from qSet to qDes
		k = 0
		while (self.cmdCount == cmdCountCur) and not rospy.is_shutdown() and k < len(qTraj[0]):
			# running setpoint
			self.qSet = [qTraj[i][k] for i in xrange(N)]
			k = k+1
                        time.sleep(T)

	def followVelocity(self, qDot, cmdCountCur):
		'''
		Follows the joint velocity setpoint as long as :
		- the corresponding position lies inside the joint limits
		- no other command has been received
		'''
		
		# ensures max velocity
		for i in xrange(N):
			if qDot[i] > jointVelMax[i]:
				qDot[i] = jointVelMax[i]
			elif -qDot[i] > jointVelMax[i]:
				qDot[i] = -jointVelMax[i]
		k = 0
		q0 = self.qSet
		while (self.cmdCount == cmdCountCur) and not rospy.is_shutdown():
			k = k+1
			# running setpoint
			self.qSet = inJointLimits([q0[i]+k*qDot[i] for i in xrange(N)])
                        time.sleep(T)
	
  
	def readBridgeCommand(self, data):
		'''
		Execute command received on /robot/command topic
		'''
		self.cmdCount += 1
		
		if sum([vi*vi for vi in data.velocity]) == 0:
			if len(data.position) != N:
				self.msgPrint = 'Bad dimension: received %i positions for %i joints' %(len(data.position), N)
			else:
				# read positions
				self.msgPrint = 'Following position #', self.cmdCount
				thread=threading.Thread(group=None,target=self.followPosition, name=None, args=(inJointLimits(data.position), self.cmdCount), kwargs={})
				thread.start()
		else:
			if len(data.velocity) != N:
				self.msgPrint = 'Bad dimension: received %i velocities for %i joints' %(len(data.velocity), N)
			else:
				# read velocities
				self.msgPrint = 'Following velocity #', self.cmdCount
				thread=threading.Thread(group=None,target=self.followVelocity, name=None, args=([v*T for v in data.velocity], self.cmdCount), kwargs={})
				thread.start()
			
if __name__ == '__main__':
	'''
	Begin of main code
	'''
	state = State()
	
	# name of the node
        rospy.init_node('joint_control')
	
	# subscribe to position and velocity command from main code
        rospy.Subscriber('/main_control/command', JointState, state.readBridgeCommand)
				
        # publish position command depending on the simulator type
        cmdPub = rospy.Publisher('/joint_control/position', JointState)
	# create JointState object - used in rviz / ViSP
	jointState = JointState()
	jointState.position = [0.]*N
	jointState.name = jointNames

	print 'Waiting commands'
	
	thread=threading.Thread(group=None,target=state.printThread, name=None, args=(), kwargs={})
	thread.start()
        silentCount = 0
        oldCount = 0
        t0Plat = time.time()
        while not rospy.is_shutdown():
                if state.cmdCount > 0 and silentCount < 10:
                        # publish current position command to the simulator
                        jointState.position = state.qSet
                        cmdPub.publish(jointState)
                        if state.cmdCount != oldCount:
                                oldCount = state.cmdCount
                                silentCount = 0
                        else:
                                silentCount += 1
                elif silentCount != 0:
                        silentCount = 0
                        state.cmdCount = 0
                        oldCount = 0
                        print 'Pausing publication'

		# should not do any other thing
		#print 'listening for new command'
		rospy.sleep(T)
