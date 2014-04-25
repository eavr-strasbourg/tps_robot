#!/usr/bin/env python

'''
Created on: 11 Oct 2013
Author: Olivier Kermorgant

Updates project files according to ROS version and environment changes
'''
import os
from commands import getoutput
from sys import argv


# Changes to ~/.bashrc: add ViSP environment variables
# Look for some files to get the path and set environment variable
with open(os.path.expanduser('~/.bashrc'), 'r+') as f:
	data = f.read()
	updateF = False
	data += '\n'
	if 'VISP_SCENES_DIR' not in data:
		thePath = os.path.dirname(getoutput("locate wireframe-simulator/camera.bnd").splitlines()[0])
		data += 'export VISP_SCENES_DIR=' + thePath + '\n'
		print 'Updating .bashrc for VISP_SCENES_DIR'
		updateF = True
	if 'VISP_ROBOT_ARMS_DIR' not in data:
		thePath = os.path.dirname(getoutput("locate robot-simulator/viper850_arm1.bnd").splitlines()[0])
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
		
		
# Archive previous works
