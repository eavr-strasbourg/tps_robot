#!/usr/bin/env python


'''
C-code generation for Denavit-Hartenberg parameters

python dh_code.py file.yml (from yaml file)
python dh_code.py file.urdf base effector (from URDF file)

author: Olivier Kermorgant
'''

import yaml
from lxml import etree
import sys, os
import sympy
from sympy.parsing.sympy_parser import parse_expr
from pylab import pi, array, norm
import re
from multiprocessing import Pool
import argparse

# Utility functions and variables
X = sympy.Matrix([1,0,0]).reshape(3,1)
Y = sympy.Matrix([0,1,0]).reshape(3,1)
Z = sympy.Matrix([0,0,1]).reshape(3,1)


def sk(u):
    return sympy.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])

def Rot(theta,u):
    R = sympy.cos(theta)*sympy.eye(3) + sympy.sin(theta)*sk(u) + (1-sympy.cos(theta))*(u*u.transpose())
    return sympy.Matrix(R)

def Rxyz(rpy):
    '''
    X - Y - Z convention, so multiply the matrices in reversed order
    '''
    return Rot(rpy[2],Z)*Rot(rpy[1],Y)*Rot(rpy[0],X)
    #return Rot(rpy[0],X)*Rot(rpy[1],Y)*Rot(rpy[2],Z)

def Homogeneous(t, R):
    #try:
    M = simp_matrix((R.row_join(t)).col_join(sympy.Matrix([[0,0,0,1]])))
    #except:
    #    M = (Rot(theta,u).row_join(t*u)).col_join(sympy.Matrix([[0,0,0,1]]))
    return M

class Bunch(object):
    def __init__(self, adict):
        self.__dict__.update(adict)

def load_yaml(filename):
    with open(filename) as f:
        d = f.read()
        print d
        robot = yaml.load(d)
        robot['keys'] = [k for k in robot]
        robot = Bunch(robot)
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
    
    prism = []     # True if prismatic
    T = []         # relative T(i-1,i) 
    u = []         # joint axis
    
    for joint in robot.joint.itervalues():
        if type(joint[iR]) == str:
            if 'q' in joint[iR]:
                prism.append(True)
        if type(joint[iTheta]) == str:
            if 'q' in joint[iTheta]:
                prism.append(False)     
        for i in xrange(4):
            if type(joint[i]) == str:
                joint[i] = parse_expr(joint[i])
        # transformation matrix
        T.append(Homogeneous(joint[iA]*X, Rot(joint[iAlpha],X)) * Homogeneous(joint[iR]*Z, Rot(joint[iTheta],Z)))
        # joint axis, always Z in DH convention
        u.append(Z)
    return T, u, prism
    
    
def load_urdf(filename, base_frame, ee_frame):
        
    def simp_rpy(rpy):
        rpy = [parse_expr(v) for v in rpy]
        for i in xrange(3):
            for k in range(-11,12):
                if abs(rpy[i] - k*pi/12.) < 1e-5:
                    if k != 0:
                        print '  changing', rpy[i], 'to', sympy.simplify(k*sympy.pi/12)
                    rpy[i] = str(sympy.simplify(k*sympy.pi/12))
                    if rpy[i] == '0':
                        rpy[i] = 0
                    break
        return rpy
    
    def simp_xyz(xyz):
        xyz = [parse_expr(v) for v in xyz]
        for i in xrange(3):
            for v in (-1,0,1):
                if abs(xyz[i]-v) < 1e-5:
                    xyz[i] = v
        return sympy.Matrix(xyz).reshape(3,1)
    
    with open(filename) as f:
        robot = etree.fromstring(f.read())
        
    # find all joints
    parents = []
    children = []
    all_joints = robot.findall('joint')
    for joint in all_joints:
        parents.append(joint.find('parent').get('link'))
        children.append(joint.find('child').get('link'))        
    # find path from base link to effector link
    joint_path = []
    cur_link = ee_frame
    while cur_link != base_frame:
        try:
            i = children.index(cur_link)
        except:
            print 'Could not find', cur_link, 'in joint list'
            sys.exit(0)
        
        if i in joint_path:
            print 'Error: passed 2 times through', cur_link
            sys.exit(0)
            
        joint_path.append(i)
        cur_link = parents[i]        
    joint_path.reverse()     
    
    # build robot geometry
    n = 0
    M = sympy.eye(4)
    T = []
    prism = []
    u = []
    parent = base_frame
    bM0 = wMe = None
    last_moving = 0
    joints = [all_joints[i] for i in joint_path]
    
    for k, joint in enumerate(joints):
        
        # get this transform
        xyz = simp_xyz(joint.find('origin').get('xyz').split(' '))
        rpy = simp_rpy(joint.find('origin').get('rpy').split(' '))
        Mi = Homogeneous(xyz, Rxyz(rpy))
        #print 'from', joint.find('parent').get('link'), 'to', child
        #print Mi
        
        if joint.get('type') != 'fixed':            
            last_moving = k
            if n == 0 and k != 0:
                # there were some fixed joints before this one, build a constant matrix bM0
                bM0 = M
                M = Mi
                print 'Constant matrix bM0 between', base_frame, 'and', joints[k-1].find('child').get('link')
            else:
                M = M*Mi
            n += 1
            #print 'joint', n, ': from', parent, 'to', child
            #print M
            # prismatic?
            prism.append(joint.get('type') == 'prismatic')
            # axis
            ax = array([float(v) for v in joint.find('axis').get('xyz').split(' ')])
            ax = ax/norm(ax)
            u.append(simp_xyz([str(v) for v in ax]))
            # Transform matrix
            q = sympy.Symbol('q%i'%n)
            if prism[-1]:
                T.append(M * Homogeneous(q*u[-1], Rot(0, X)))
            else:
                T.append(M * Homogeneous(0*X, Rot(q, u[-1])))
            # reset M for next joint
            M = sympy.eye(4)
        else:
            M = M*Mi
            
    if joint.get('type') == 'fixed':
        # we finished on some fixed links
        wMe = M       
        print 'Constant matrix wMe between', joints[last_moving].find('child').get('link'), 'and', ee_frame
    return T, u, prism, bM0, wMe


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

def compute_Ji(joint_prism, u0, p0, i):
    '''
    Compute the i-eth column of the Jacobian (used for multiprocessing)
    '''
    if joint_prism[i]:
        # prismatic joint: v = qdot.u and w = 0
        Jv = simp_matrix(u0[i])
        Jw = sympy.Matrix([[0,0,0]]).reshape(3,1)
    else:
        # revolute joint: v = [qdot.u]x p and w = qdot.u
        Jv = simp_matrix(sk(u0[i])*(p0[-1]-p0[i]))
        Jw = simp_matrix(u0[i])
    print '   J_%i' % (i+1)
    return (i, Jv.col_join(Jw))    # register this column as column i
    #return Jv.col_join(Jw)

def cCode(s):
    '''
    Prints C++ code
    '''
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
    '''
    Replace cos and sin functions of q_i with precomputed constants
    '''
    fctList = ('cos', 'sin')
    pmDict = {'+':'p', '-':'m'}
    defCount = len(cDef)
    prtDebug('len', defCount)
    for i in xrange(1,dof+1):
        for fct in fctList:
            # look for simple calls ie cos(q1)
            sf = '%s(q%i)' % (fct, i)                                                       # cos(q1)
            if s.find(sf) > -1:
                prtDebug('found', sf)
                if sf not in cDef:
                    sUse = '%s(%s[%i])' % (fct, args.q, i-1)                                # cos(q[0])
                    cUse[sf] = '%s%i' % (fct[0],i)                                          # c1
                    cDef[sf] = 'const double %s = %s;' % (cUse[sf], sUse)                   # const double c1 = cos(q[0]);
                    prtDebug('   newUse :', cUse[sf])
                    prtDebug('   defined:', cDef[sf])
                    prtDebug('   cUse: ', len(cUse))
                    prtDebug('   cDef: ', len(cDef))
                s = s.replace(sf, cUse[sf])                                                 # replace cos(q1)
            # look for double calls ie cos(q1 + q2) or sin(q1 - q2)
            for j in xrange(i+1, dof+1):
                for pm in pmDict:
                    sf = '%s(q%i %s q%i)' % (fct, i, pm, j)                                 # cos(q1 + q2)
                    if s.find(sf) > -1:
                        prtDebug('found', sf)
                        sUse = '%s(%s[%i]+%s[%i])' % (fct, args.q, i-1, args.q, j-1)        # cos(q[0]+q[1])
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
                sUse = '%s[%i]' % (args.q, i-1)                                                                                                     # q[0]  
                s = s.replace(sf, sUse)                                                                                                         # replace q1 (remaining prismatic joints, optim not really useful)
    return s.replace('1.00000000000000','1'), cDef, cUse

def exportCpp(M, s='M',cDef={},cUse={}):
        '''
        Writes the C++ code corresponding to a given matrix
        '''
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
                        ms, cDef, cUse = replaceFctQ(str(sympy.N(M[i,j])), cDef, cUse)
                        out.append(s + sRows + sCols + ' = ' + ms + ';')
        return out, cDef, cUse

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'A module to generate C++ code from URDF or Yaml (DH) file.'

    # files
    parser.add_argument('files', metavar='file', type=str, nargs='+', help='File (and base and end-effector frames for URDF)')

    parser.add_argument('-q', metavar='q', help='How the joint vector appears in the code',default='q')
    parser.add_argument('-T', metavar='T', help='How the pose matrix appears in the code',default='T')
    parser.add_argument('-J', metavar='J', help='How the Jacobian matrix appears in the code',default='J') 
    parser.add_argument('--all_J', action='store_true', help='Computes the Jacobian of all frames',default=False)
    parser.add_argument('--only-fixed', action='store_true', help='Only computes the fixed matrices, before and after the arm',default=False)
    args = parser.parse_args()

    # check robot description file
    if not os.path.lexists(args.files[0]):
            print 'File', args.files[0], 'does not exist'
            sys.exit(0)
    bM0 = wMe = None
    
    # load into symbolic
    print ''
    print 'Building intermediary matrices...'
    if args.files[0][-4:] == 'urdf':
        if len(args.files) == 3:
            T, u, prism, bM0, wMe = load_urdf(args.files[0], args.files[1], args.files[2])
        else:
            print 'Not enough arguments for URDF parsing - frames needed'
            sys.exit(0)
    elif args.files[0][-4:] == 'yaml' or args.files[0][-3:] == 'yml':
        T, u, prism = load_yaml(args.files[0])
    else:
        print 'Unknown file type', args.files[0]

    # get number of joints
    dof = len(T)
    
    # Transform matrices
    if not args.only_fixed:
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
        # Rotation of each frame to go to frame 0
        print ''
        print 'Building differential kinematic model...'
        
        R0 = [M[:3,:3] for M in T0]
        # joint axis expressed in frame 0
        u0 = [R0[i]*u[i] for i in xrange(dof)] 
        
        all_J = []
        
        if args.all_J:
            ee_J = range(1,dof+1)
        else:
            ee_J = [dof]
        for ee in ee_J:
            # origin of each frame expressed in frame 0
            p0 = [T0[i][:3,3] for i in xrange(ee)]
                        
            # build Jacobian
            pool = Pool()
            results = []
            for i in xrange(ee):
                # add this column to pool
                results.append(pool.apply_async(compute_Ji, args=(prism, u0, p0, i)))

            Js = sympy.Matrix()
            Js.rows = 6
            iJ = [result.get() for result in results]
            for i in xrange(ee):
                for iJi in iJ:
                    if iJi[0] == i:
                        Js = Js.row_join(iJi[1])
            all_J.append(Js)
        print ''
        pool.terminate()

        print ''
        print 'Building pose C code...'
        Plines, Pdef, Puse = exportCpp(T0[-1], args.T)
        print ''
        print '    // Generated pose code'
        cCode(Pdef)
        cCode(Plines)
        print '    // End of pose code'

        print ''
        print 'Building Jacobian C code...'
        if args.all_J:
            for i,Js in enumerate(all_J):
                Jlines, Jdef, Juse = exportCpp(Js, args.J + str(i+1))
                print ''
                print '    // Generated Jacobian code to link %i'% (i+1)
                cCode(Jdef)
                cCode(Jlines)
                print '    // End of Jacobian code to link %i'% (i+1)
        else:
            Jlines, Jdef, Juse = exportCpp(Js, args.J)
            print ''
            print '    // Generated Jacobian code'
            cCode(Jdef)
            cCode(Jlines)
            print '    // End of Jacobian code'        
    
    fixed_M = ((wMe, 'wMe','end-effector'), (bM0,'bM0','base frame'))
    for M in fixed_M:
        if M[0] != None:
            print ''
            print 'Building %s code...' % M[1]
            Mlines, Mdef, Muse = exportCpp(M[0], M[1])
            print ''
            print '    // Generated %s code' % M[2]
            cCode(Mlines)
            print '    // End of %s code' % M[2]
            
    
