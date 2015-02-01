#!/usr/bin/env python

'''
Created on: 11 Oct 2013
Author: Olivier Kermorgant

Archive les anciens TP et nettoie les repertoires de compilation
'''
import os
from roslaunch import substitution_args
import shutil

# Cherche le repertoire src
path_src = substitution_args.resolve_args('$(find tps_robot)')
path_src = '/'.join(path_src.split('/')[:-1]) + '/'
path_devel = path_src.replace('/src/', '/devel/')
path_build = path_src.replace('/src/', '/build/')

# Cree un repertoire d'archives
path_arch = path_src.replace('/src/', '/src_archived/')
if os.path.lexists(path_arch) == False:
    os.mkdir(path_arch)

# Cherche les anciens packages de TP et deplace vers les archives
user_pkg = os.listdir(path_src)
user_pkg = [pkg for pkg in user_pkg if pkg != 'tps_robot'] 

for pkg in user_pkg:
    goes_to_archive = False
    if os.path.lexists(path_src + pkg + '/package.xml'):
        with open(path_src + pkg + '/package.xml') as pkg_xml:
            if 'tps_robot' in pkg_xml.read():
                goes_to_archive = True
    if goes_to_archive:
        os.chdir(path_src)
        try:
            pkg_arch = shutil.make_archive(pkg, 'tar', path_src, pkg)
            shutil.move(pkg_arch, path_arch)
            print 'removing', path_src+pkg
            shutil.rmtree(path_src+pkg)
        except:
            pass
                
# Nettoie le repertoire de build
for elem in os.listdir(path_build):
    if os.path.isdir(path_build + elem):
        shutil.rmtree(path_build + elem)
    else:
        os.remove(path_build + elem)
# Nettoie le repertoire de devel
for elem in os.listdir(path_devel):
    if os.path.isdir(path_devel + elem):
        shutil.rmtree(path_devel + elem)