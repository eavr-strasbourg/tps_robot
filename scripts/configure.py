#!/usr/bin/env python

'''
Created on: 11 Oct 2013
Author: Olivier Kermorgant

Updates project files according to ROS version and environment changes
'''
import os
from commands import getoutput
from sys import argv

rosVersion = 'fuerte'
if len(argv) > 1:
	if argv[1].lower() in ('fuerte', 'groovy', 'hydro'):
		rosVersion = argv[1].lower()
	else:
		print rosVersion, 'not recognized as ROS version'

if rosVersion not in os.listdir('/opt/ros'):
	print rosVersion, 'not in /opt/ros/'
	rosVersion = os.listdir('/opt/ros')[0]
	
print 'Configuring for', rosVersion

# Changes to ~/.bashrc: add ViSP environment variables
# Look for some files to get the path and set environment variable
with open(os.path.expanduser('~/.bashrc'), 'r+') as f:
	data = f.read()
	updateF = False
	data += '\n'
	if 'VISP_SCENES_DIR' not in data:
		thePath = os.path.dirname(getoutput("locate visp/install/share/visp/data/wireframe-simulator/camera.bnd").splitlines()[0])
		data += 'export VISP_SCENES_DIR=' + thePath + '\n'
		print 'Updating .bashrc for VISP_SCENES_DIR'
		updateF = True
	if 'VISP_ROBOT_ARMS_DIR' not in data:
		thePath = os.path.dirname(getoutput("locate visp/install/share/visp/data/robot-simulator/viper850_arm1.bnd").splitlines()[0])
		data += 'export VISP_ROBOT_ARMS_DIR=' + thePath + '\n'
		print 'Updating .bashrc for VISP_ROBOT_ARMS_DIR'
		updateF = True
	if updateF:
		f.seek(0)
		f.truncate()
		f.write(data)
		print '.bashrc has been updated - launch a new console'
	else:
		print 'Nothing to do in .bashrc'

# Changes to launchfile to get the right config file in RViz (.vcg for fuerte, .rviz for groovy and next)
lfPath = os.path.dirname(os.path.abspath(argv[0])) + '/../launch/'
launchFiles = os.listdir(lfPath)
for lf in launchFiles:
	if '.launch' in lf:
		updateF = False
		with open(lfPath + lf, 'r+') as f:
			data = f.read().splitlines()
			for i,di in enumerate(data):
				if 'rviz' in di and 'launch/config' in di:
					if rosVersion == 'fuerte' and 'config.rviz' in di:
						data[i] = data[i].replace('config.rviz', 'config.vcg')
						updateF = True
					elif rosVersion != 'fuerte' and 'config.vcg' in di:
						data[i] = data[i].replace('config.vcg', 'config.rviz')
						updateF = True
			if updateF:
				print 'Updating', lf
				f.seek(0)
				f.truncate()
				f.write('\n'.join(data))
			else:
				print 'Nothing to do in', lf
			