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
import multiprocessing as mp

# human sorting
def human_sort(l):
        convert = lambda text: int(text) if text.isdigit() else text 
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
        l.sort( key=alphanum_key )

debug = False

def prtDebug(s1, s2=''):
        if debug:
                print s1, s2


def simp_matrix(M):
    '''
    simplify matrix for old versions of sympy
    '''
    for i in xrange(M.rows):
        for j in xrange(M.cols):
            M[i,j] = sympy.simplify(M[i,j])
    return M    

def compute_Ji(joint_prism, u, p, i, output):
    if joint_prism[i]:
        Jv = simp_matrix(u[i])
        Jw = sympy.Matrix([[0,0,0]]).reshape(3,1)
    else:
        Jv = simp_matrix(sk(u[i])*(p[dof-1]-p[i-1]))
        Jw = simp_matrix(u[i])
    print '   J_%i' % (i+1)
    output.put((i, Jv.col_join(Jw)))
    #return Jv.col_join(Jw)

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
    
    X = sympy.Matrix([1,0,0]).reshape(3,1)
    Y = sympy.Matrix([0,1,0]).reshape(3,1)
    Z = sympy.Matrix([0,0,1]).reshape(3,1)
    
    def sk(u):
        return sympy.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])
    
    def Rot(theta,u):
        R = sympy.cos(theta)*sympy.eye(3) + sympy.sin(theta)*sk(u) + (1-sympy.cos(theta))*(u*u.transpose())
        return sympy.Matrix(R)
    
    def Rxyz(rpy):
        return Rot(rpy[0],X)*Rot(rpy[1],Y)*Rot(rpy[2],Z)

    def Homogeneous(t, R):
        #try:
        M = simp_matrix((R.row_join(t)).col_join(sympy.Matrix([[0,0,0,1]])))
        #except:
        #    M = (Rot(theta,u).row_join(t*u)).col_join(sympy.Matrix([[0,0,0,1]]))
        return M
        
    sq = 'q'
    sP = 'P'
    sM = 'T'
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
            
    # get convention
    conv = 'dh'
    if 'convention' in robot.keys:
        conv = robot.convention

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
    print ''
    print 'Building intermediary matrices...'
    
    joint_prism = []     # 0 = revolute, 1 = prismatic
    T = []               # relative T(i-1,i) 
    joint_u = []         # joint axis
    
    if conv == 'dh':
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
            # fixed matrix
            T.append(Homogeneous(joint[iA]*X, Rot(joint[iAlpha],X)) * Homogeneous(joint[iR]*Z, Rot(joint[iTheta],Z)))
            # joint axis
            joint_u.append(Z)
    elif conv == 'urdf':
        for i in robot.joint:
            joint = robot.joint[i]
            # joint axis
            joint_u.append(sympy.Matrix([parse_expr(str(v)) for v in joint['axis']]).reshape(3,1))
            # joint type
            joint_prism.append(joint['type'] == 'prismatic')
            # fixed matrix
            xyz = sympy.Matrix([parse_expr(str(v)) for v in joint['xyz']]).reshape(3,1)
            rpy = [parse_expr(str(v)) for v in joint['rpy']]
            q = sympy.Symbol('q%i'%i)
            if joint_prism[-1]:
                T.append(Homogeneous(xyz, Rxyz(rpy)) * Homogeneous(q*joint_u[-1], Rot(0, X)))
            else:
                T.append(Homogeneous(xyz, Rxyz(rpy)) * Homogeneous(0*X, Rot(q, joint_u[-1])))
    else:
        print 'Unknown convention', conv
        sys.exit(0)                                       
                
                
    # Transform matrices
    print ''
    print 'Building direct kinematic model...'    
    T0 = []     # absolute T(0,i)
    for i in xrange(dof):
        if len(T0) == 0:
            T0.append(T[i])
        else:
            T0.append(simp_matrix(T0[-1]*T[i]))
            print '  T %i/0' % (i+1)
             
    # Jacobian   
    # Rotation part
    print ''
    print 'Building differential kinematic model...'
    R0 = [M[:3,:3] for M in T0]    
    # origin part
    p = [T0[i][:3,3] for i in xrange(dof)]
    p.append(sympy.Matrix([[0],[0],[0]]))
    u = [R0[i]*joint_u[i] for i in xrange(dof)] # joint axis expressed in base frame
    # build Jacobian
    
    output = mp.Queue()
    processes = [mp.Process(target=compute_Ji, args=(joint_prism, u, p, i, output)) for i in xrange(dof)]
    for proc in processes:
        proc.start()
    for proc in processes:
        proc.join()

    Js = sympy.Matrix()
    Js.rows = 6
    iJ = [output.get() for proc in processes]
    for i in xrange(dof):
        for iJi in iJ:
            if iJi[0] == i:
                Js = Js.row_join(iJi[1])
    print ''
   
    print ''
    print 'Building pose C code...'
    if 'pose' in robot.keys:
        P = sympy.Matrix([0]*len(robot.pose))
        # End-effector pose XYZ RPY
        Ps = list(T0[-1][:3,3])                                                                                 # XYZ
        Ps.append(sympy.atan2(-T0[-1][0,2],T0[-1][1,2]))                                                           # Roll
        Ps.append(sympy.atan2(T0[-1][0,2]*sympy.sin(Ps[3])-T0[-1][1,2]*sympy.cos(Ps[3]),T0[-1][2,2]))                   # Pitch
        Ps.append(sympy.atan2(-T0[-1][0,1]*sympy.cos(Ps[3])-T0[-1][1,1]*sympy.sin(Ps[3]),T0[-1][0,0]*sympy.cos(Ps[3])+T0[-1][1,0]*sympy.sin(Ps[3]))) # Yaw
    
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
    
    fixed_M = {'wMe':'end-effector', 'bM0':'base frame'}
    for key in fixed_M:
        if key in robot.keys:
            print ''
            print 'Building %s code...' % fixed_M[key]
            xyz = sympy.Matrix([parse_expr(str(v)) for v in robot.wMe['xyz']]).reshape(3,1)
            rpy = [parse_expr(str(v)) for v in robot.wMe['rpy']]
            Mlines, Mdef, Muse = exportCpp(Homogeneous(xyz, Rxyz(rpy)), 'wMe')
            print ''
            print '    // Generated %s code' % fixed_M[key]
            cCode(Mlines)
            print '    // End of %s code' % fixed_M[key]
