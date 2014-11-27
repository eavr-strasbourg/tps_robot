#!/usr/bin/env python


'''
C-code generation for Denavit-Hartenberg parameters

python dh_code.py file.yml

author: Olivier Kermorgant
'''

import yaml
import sys
import sympy
from sympy.parsing.sympy_parser import parse_expr
from pylab import concatenate
import re

# human sorting
def human_sort(l):
        convert = lambda text: int(text) if text.isdigit() else text 
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
        l.sort( key=alphanum_key )

debug = False

def prtDebug(s1, s2=''):
        if debug:
                print s1, s2

class Bunch(object):
  def __init__(self, adict):
    self.__dict__.update(adict)

def cCode(s):
        print ''
        if type(s) == list:
                for line in s:
                        print '   ',line
        else:
                v = s.values()
                human_sort(v)
                for line in v:
                        print '   ',line

def replaceFctQ(s, cDef, cUse):
        # look for functions, append const definition
        fctList = ('cos', 'sin')
        pmDict = {'+':'p', '-':'m'}
        defCount = len(cDef)
        prtDebug('len', defCount)
        for i in xrange(1,dof+1):
                for fct in fctList:
                        # look for simple calls ie cos(q1)
                        sf = '%s(q%i)' % (fct, i)                                                                                                       # cos(q1)
                        if s.find(sf) > -1:
                                prtDebug('found', sf)
                                if sf not in cDef:
                                        sUse = '%s(%s[%i])' % (fct, sq, i-1)                                                            # cos(q[0])
                                        cUse[sf] = '%s%i' % (fct[0],i)                                                                  # c1
                                        cDef[sf] = 'const double %s = %s;' % (cUse[sf], sUse)                   # const double c1 = cos(q[0]);
                                        prtDebug('   newUse :', cUse[sf])
                                        prtDebug('   defined:', cDef[sf])
                                        prtDebug('   cUse: ', len(cUse))
                                        prtDebug('   cDef: ', len(cDef))
                                s = s.replace(sf, cUse[sf])                                                                                             # replace cos(q1)
                        # look for double calls ie cos(q1 + q2) or sin(q1 - q2)
                        for j in xrange(i+1, dof+1):
                                for pm in pmDict:
                                        sf = '%s(q%i %s q%i)' % (fct, i, pm, j)                                                         # cos(q1 + q2)
                                        if s.find(sf) > -1:
                                                prtDebug('found', sf)
                                                sUse = '%s(%s[%i]+%s[%i])' % (fct, sq, i-1, sq, j-1)                    # cos(q[0]+q[1])
                                                if sf not in cDef:
                                                        cUse[sf] = '%s%i%s%i' % (fct[0],i,pmDict[pm],j)                 # c1p2
                                                        cDef[sf] = 'const double %s = %s;' % (cUse[sf],sUse)    # const double c1p2 = cos(q[0]+q[1]);
                                                        prtDebug('   newUse :', cUse[sf])
                                                        prtDebug('   defined:', cDef[sf])
                                                        prtDebug('   cUse: ', len(cUse))
                                                        prtDebug('   cDef: ', len(cDef))
                                                s = s.replace(sf, cUse[sf])                                                                             # replace cos(q1 + q2)
                
                # look for other occurences of q
                sf = 'q%i' % i                                                                                                                                  # q1
                if s.find(sf) > -1:
                        sUse = '%s[%i]' % (sq, i-1)                                                                                                     # q[0]  
                        s = s.replace(sf, sUse)                                                                                                         # replace q1 (remaining prismatic joints, optim not really useful)
        return s, cDef, cUse

def exportCpp(M, s='M',cDef={},cUse={}):
        out = []

        # write each element
        sRows = ''
        sCols = ''
        for i in xrange(M.rows):
                if M.rows > 1:
                        sRows = '[' + str(i) + ']'
                for j in xrange(M.cols):
                        if M.cols > 1:
                                sCols = '[' + str(j) + ']'
                        ms, cDef, cUse = replaceFctQ(str(sympy.simplify(M[i,j])), cDef, cUse)
                        out.append(s + sRows + sCols + ' = ' + ms + ';')
        return out, cDef, cUse
    

if __name__ == '__main__':
    
    def sk(u):
        return sympy.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])
    
    def Rot(theta,u):
        R = sympy.cos(theta)*sympy.eye(3) + sympy.sin(theta)*sk(u) + (1-sympy.cos(theta))*(u*u.transpose())
        return sympy.Matrix(R)

    def Homogeneous(t, theta, u):
        return sympy.Matrix(concatenate((concatenate((Rot(theta,u), t*u),1), sympy.Matrix([[0,0,0,1]])),0))
    
    
    sq = 'q'
    sP = 'P'
    sM = 'M'
    sJ = 'J'

    # load robot description file (Yaml)
    print ''
    for arg in sys.argv:
            if '.yml' in arg:
                    with open(arg) as f:
                            d = f.read()
                            print d
                            robot = yaml.load(d)
                            robot['keys'] = [k for k in robot]
                            robot = Bunch(robot)

    # get number of joints
    dof = len(robot.joint)

    # get robot name
    name = 'robot'
    if 'name'in robot.keys:
            name = robot.name

    # get notations
    if 'q' in robot.keys:
            sq = robot.q
    if 'P' in robot.keys:
            sP = robot.P
            sM = robot.P
    if 'J' in robot.keys:
            sJ = robot.J

    # get ordering
    if 'notation' in robot.keys:
            iAlpha = robot.notation.index('alpha')
            iA = robot.notation.index('a')
            iR = robot.notation.index('r')
            iTheta = robot.notation.index('theta')
    else:
            iAlpha = 0
            iA = 1
            iR = 2
            iTheta = 3
    # change into symbolic
    joint_prism = []
    for joint in robot.joint.itervalues():
        if type(joint[iR]) == str:
            if 'q' in joint[iR]:
                joint_prism.append(1)
        if type(joint[iTheta]) == str:
            if 'q' in joint[iTheta]:
                joint_prism.append(0)
        for i in xrange(4):
            if type(joint[i]) == str:
                joint[i] = parse_expr(joint[i])
            
                
    print ''
    
    print 'Building model...'
    # transform matrices
    X = sympy.Matrix([1,0,0]).reshape(3,1)
    Z = sympy.Matrix([0,0,1]).reshape(3,1)

    # Transform matrices
    T = []      # relative T(i-1,i) 
    T0 = []     # absolute T(0,i)
    for joint in robot.joint.itervalues():
        T.append(sympy.simplify(Homogeneous(joint[iA], joint[iAlpha],X) * Homogeneous(joint[iR], joint[iTheta],Z)))
        if len(T0) == 0:
            T0.append(T[-1])
        else:
            T0.append(T0[-1]*T[-1])
        
    # End-effector pose XYZ RPY
    Ps = list(T0[-1][:3,3])                                                                                 # XYZ
    Ps.append(sympy.atan2(-T0[-1][0,2],T0[-1][1,2]))                                                           # Roll
    Ps.append(sympy.atan2(T0[-1][0,2]*sympy.sin(Ps[3])-T0[-1][1,2]*sympy.cos(Ps[3]),T0[-1][2,2]))                   # Pitch
    Ps.append(sympy.atan2(-T0[-1][0,1]*sympy.cos(Ps[3])-T0[-1][1,1]*sympy.sin(Ps[3]),T0[-1][0,0]*sympy.cos(Ps[3])+T0[-1][1,0]*sympy.sin(Ps[3]))) # Yaw
        
    # Jacobian   
    # Rotation part
    R0 = [M[:3,:3] for M in T0]    
    # origin part
    p = []
    for i in xrange(dof):
        p.append(T0[i][:3,3])
    p.append(sympy.Matrix([[0],[0],[0]]))
    z = [R*Z for R in R0]
    # build Jacobian
    Js = sympy.Matrix()
    Js.rows = 6
    for i in xrange(dof):
        Jv = joint_prism[i]*z[i] + (1-joint_prism[i])*sk(z[i])*(p[dof-1]-p[i-1])
        Jw = (1-joint_prism[i]) * z[i]
        Js = sympy.Matrix(concatenate((Js,sympy.Matrix(concatenate((Jv,Jw),0))),1))
      
    print ''
    print 'Building pose C code...'
    if 'pose' in robot.keys:
        P = sympy.Matrix([0]*len(robot.pose))
        for i,p in enumerate(robot.pose):
                if p == 'x':
                        P[i] = Ps[0]
                elif p == 'y':
                        P[i] = Ps[1]
                elif p == 'z':
                        P[i] = Ps[2]
                elif p[0] == 'q':
                        P[i] = sympy.Symbol(p)
                else:
                        print 'Pose parameter not recognized:', p
        Plines, Pdef, Puse = exportCpp(P, sP)
    else:
            Plines, Pdef, Puse = exportCpp(T0[-1], sM)
    print ''
    print '    // Generated pose code'
    cCode(Pdef)
    cCode(Plines)
    print '    // End of pose code'

    print ''
    print 'Building Jacobian C code...'
    if 'pose' in robot.keys:
        J = sympy.Matrix([[0]*len(robot.pose)] * dof)
        for i,p in enumerate(robot.pose):
                if p == 'x':
                        J[i,:] = Js[0,:]
                elif p == 'y':
                        J[i,:] = Js[1,:]
                elif p == 'z':
                        J[i,:] = Js[2,:]
                elif p[0] == 'q':
                        J[i,int(p[1:])-1] = 1
                else:
                        print 'Pose parameter not recognized:', p
    else:
        J = Js
    Jlines, Jdef, Juse = exportCpp(J, sJ)
    print ''
    print '    // Generated Jacobian code'
    cCode(Jdef)
    cCode(Jlines)
    print '    // End of Jacobian code'
    
