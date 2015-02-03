#!/usr/bin/env python

'''
Created on: 11 Oct 2013
Author: Olivier Kermorgant

Installe ROS et met en place les repertoires pour le TP 
A lancer avec sudo afin d'installer les packages via apt-get
'''
import os, sys
from platform import linux_distribution
from subprocess import call
from getpass import getuser

if getuser() == 'root':
	print 'Ce script est a lancer en mode utilisateur'
	sys.exit(0)
	
USER_HOME = '/home/'+getuser()

UBUNTU_VERSION = linux_distribution()[2]
DEBUT_CONFIG = '# --- Configuration ROS ---'
FIN_CONFIG = '# --- Fin configuration ROS ---'

ROSVERSION = 'indigo'
if UBUNTU_VERSION == 'precise':
    ROSVERSION = 'hydro'

    
def user_call(cmd_line):
    '''
    Call command line as user
    '''
    print 'calling:', cmd_line
    call(cmd_line.split(' '))
    
# Installation
user_call('sh install_ros.sh %s %s' % (UBUNTU_VERSION, ROSVERSION))

# Environnement et dossiers utilisateur
print "[Configuration de l'environnement]"
os.chdir(USER_HOME)
with open('.bashrc', 'r+') as f:
    data = f.read().splitlines()
    # Effacement configuration precedente
    try:
        ind_debut = data.index(DEBUT_CONFIG)
        ind_fin = data.index(FIN_CONFIG)
        for i in xrange(ind_debut, ind_fin+1):
            data.pop(ind_debut)
    except:
        pass
    # Construction nouvelle configuration
    data.append(DEBUT_CONFIG)
    data.append('source /opt/ros/%s/setup.bash' % ROSVERSION)
    data.append('source ~/ros/devel/setup.bash')
    data.append('export VISP_ROBOT_ARMS_DIR=/opt/ros/%s/share/visp/data/robot-simulator' % ROSVERSION)
    data.append('export VISP_SCENES_DIR=/opt/ros/%s/share/visp/data/wireframe-simulator' % ROSVERSION)
    data.append(FIN_CONFIG)
    
    # Ecriture
    f.seek(0)
    f.write('\n'.join(data))
    
call(['bash','-c', 'source ~/.bashrc'])

print '[Telechargement sujet de TP robotique]'
os.chdir(USER_HOME+'/ros/src')
user_call('git clone https://github.com/eavr-strasbourg/tps_robot.git')


print '' 
print "[Pret pour utilisation !!]"
print "Relancer un nouveau terminal"
print ''
