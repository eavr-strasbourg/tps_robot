#!/usr/bin/env python

'''
Created on: 11 Oct 2013
Author: Olivier Kermorgant

Installe ROS et met en place les repertoires pour le TP 
A lancer avec sudo afin d'installer les packages via apt-get
'''
import os
from platform import linux_distribution
from commands import getoutput
from subprocess import call
from sys import argv
import argparse

UBUNTU_VERSION = linux_distribution()[2]
USER = os.getenv('SUDO_USER')
DEBUT_CONFIG = '# --- Configuration ROS ---'
FIN_CONFIG = '# --- Fin configuration ROS ---'

ROSVERSION = 'indigo'
if UBUNTU_VERSION == 'precise':
    ROSVERSION = 'hydro'


if USER == '':
    print "Lancer ce script avec sudo pour permettre l'installation des packages via apt-get"
    sys.exit(0)
USER_HOME = 'home/%s' % USER

def root_call(cml_line):
    '''
    Call command line as root
    '''
    print '   root:', cmd_line
    call(cml_line)
    
    
def user_call(cmd_line):
    '''
    Call command line as user and not root
    Useful for file permissions
    '''
    print '   user:', cmd_line
    call('su -c %s -s /bin/sh %s' % (cmd_line, USER))
    
    
# Installation de ROS
print 'Ajout des depots ROS'
with open('/etc/apt/sources.list.d/ros-latest.list','w') as f:
    f.write('deb http://packages.ros.org/ros/ubuntu %s main' % UBUNTU_VERSION)
root_call('wget http://packages.ros.org/ros.key -O - | apt-key add -')
root_call('apt-get update')
   

print 'Installation des packages'
root_call('apt-get install -yq ros-%s-rviz ros-%s-vision-visp ros-%s-ros-comm python-rosinstall python-rosdep gitk qtcreator' % (ROSVERSION, ROSVERSION, ROSVERSION))
root_call('rosdep init')
user_call('rosdep update')

# Environnement et dossiers utilisateur
print "Configuration de l'environnement"
os.chdir(USER_HOME)
with open('.bashrc', 'r+') as f:
    data = f.read.splitlines()
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
root_call('chown %s .bashrc' % USER)
    
user_call('source /opt/ros/%s/setup.bash' % ROSVERSION)


print 'Configuration des dossiers'
user_call('mkdir -p ros_workspace/src')
os.chdir('%s/ros_workspace/src' % USER_HOME)
user_call('catkin_init_workspace')
os.chdir('%s/ros_workspace' % USER_HOME)
user_call('catkin_make')
user_call('source ~/.bashrc')

print 'Telechargement sujet de TP robotique'
os.chdir('%s/ros_workspace/src' % USER_HOME)
user_call('git clone https://github.com/eavr-strasbourg/tps_robot.git')
user_call('rospack profile')


print "Pret pour utilisation"
