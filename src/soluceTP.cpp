
#include <tps_robot/tps_robot.h>
#include <ros/ros.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTime.h>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>    // std::max
#include <visp/vpVelocityTwistMatrix.h>

using std::cout;
using std::endl;

// namespaces for robots

namespace turret
{
const double b=.5, d=.1;
const unsigned int m = 3;

// MGD 3 ddl
void calcMGD(const vpColVector &q, vpColVector &p)
{
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    p[0] = -(d + q[2])*s2*c1;
    p[1] = -(d + q[2])*s1*s2;
    p[2] = b + (d + q[2])*c2;
    p[0] = p[0];
}
// Jacobien 3 ddl
void calcJac(const vpColVector &q, vpMatrix &J)
{
    J.resize(3,3,false);
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    J[0][0] = (d + q[2])*s1*s2;
    J[0][1] = -(d + q[2])*c1*c2;
    J[0][2] = -s2*c1;
    J[1][0] = -(d + q[2])*s2*c1;
    J[1][1] = -(d + q[2])*s1*c2;
    J[1][2] = -s1*s2;
    // J[2][0] = 0;
    J[2][1] = -(d + q[2])*s2;
    J[2][2] = cos(q[1]);
}
// MGI 3 ddl
void calcMGI(const vpColVector &qi, const vpColVector &p, vpColVector &q)
{
    if(p[1] == 0 && p[0] == 0)
    {
        q[0] = qi[0];
        q[1] = 0;
        q[2] = p[2]-b-d;
    }
    else
    {
        q[0] = atan2(p[1],p[0]);
        const double l = sqrt(p[0]*p[0] + p[1]*p[1]);
        q[1] = atan2(-l, p[2]-b);
        q[2] = sqrt(l*l + vpMath::sqr(p[2]-b)) - d;

        cout << "MGI1: " << q.t() << endl;

        if(qi[1] * q[1] < 0)
        {
            q[1] *= -1;
            q[0] -= M_PI*vpMath::sign(q[0]);
        }
    }
}
}


namespace scara {

const double h=.27, a=.2, b=.15, c=.07;
const unsigned int m = 4;
// MGD 4 ddl
void calcMGD(const vpColVector &q, vpColVector &p)
{
    const double c1 = cos(q[0]);
    const double c1p2 = cos(q[0] + q[1]);
    const double c3 = cos(q[2]);
    const double s1 = sin(q[0]);
    const double s1p2 = sin(q[0] + q[1]);
    const double s3 = sin(q[2]);
    p[0] = a*c1 + b*c1p2 - (c + q[3])*s3*s1p2;
    p[1] = a*s1 + b*s1p2 + (c + q[3])*s3*c1p2;
    p[2] = h - (c + q[3])*c3;
    p[3] = q[2];
}
// Jacobien 4 ddl
void calcJac(const vpColVector &q, vpMatrix &J)
{
    J.resize(4,4,false);
    const double c1 = cos(q[0]);
    const double c1p2 = cos(q[0] + q[1]);
    const double c3 = cos(q[2]);
    const double s1 = sin(q[0]);
    const double s1p2 = sin(q[0] + q[1]);
    const double s3 = sin(q[2]);
    J[0][0] = -a*s1 - b*s1p2 - (c + q[3])*s3*c1p2;
    J[0][1] = -b*s1p2 - (c + q[3])*s3*c1p2;
    J[0][2] = -(c + q[3])*s1p2*c3;
    J[0][3] = -s3*s1p2;
    J[1][0] = a*c1 + b*c1p2 - (c + q[3])*s3*s1p2;
    J[1][1] = b*c1p2 - (c + q[3])*s3*s1p2;
    J[1][2] = (c + q[3])*c3*c1p2;
    J[1][3] = sin(q[2])*c1p2;
    J[2][0] = 0;
    J[2][1] = 0;
    J[2][2] = (c + q[3])*s3;
    J[2][3] = -c3;
    J[3][0] = 0;
    J[3][1] = 0;
    J[3][2] = 1;
    J[3][3] = 0;
}
// MGI iteratif 4 ddl
void calcMGI(const vpColVector &qi, const vpColVector &pd, vpColVector &q)
{
    q = qi;
    vpColVector p = pd;
    vpMatrix J,I;
    const double lambda = .5;
    const double eMin = .001;
    double e = 2*eMin;
    unsigned int i = 0;
    while(i++ < 1000 && e > eMin)
    {
        calcMGD(q, p);
        calcJac(q, J);
        q += -lambda * J.t() * (p-pd);
        e = (p-pd).euclideanNorm();
    }
    if(e > eMin)
        std::cout << "MGI iter: error remaining: " << e << std::endl;
}
}

void calcMGD(const vpColVector &q, vpColVector &p, const unsigned int &N)
{
    switch(N)
    {
    case 3: turret::calcMGD(q, p);break;
    case 4: scara::calcMGD(q, p);break;
    default:break;
    }
}


void calcJac(const vpColVector &q, vpMatrix &J, const unsigned int &N)
{
    switch(N)
    {
    case 3: turret::calcJac(q, J);break;
    case 4: scara::calcJac(q, J);break;
    default:break;
    }
}

void calcMGI(const vpColVector &qi, const vpColVector &pd, vpColVector &q, const unsigned int &N)
{
    switch(N)
    {
    case 3: turret::calcMGI(qi, pd, q);break;
    case 4: scara::calcMGI(qi, pd, q);break;
    default:break;
    }
}








int main(int argc, char ** argv)
{
    // init ROS
    ros::init(argc, argv, "mainControl");
    ros::start();
    ros::NodeHandle rosNH;
    double rosRate = 100;
    ros::Rate loop((int) rosRate);

    // init robot
    tpRobot robot(rosNH);
    const unsigned int N = robot.getDOFs();

    // publish setpoint / position / error
    ros::Publisher setpointPub = rosNH.advertise<std_msgs::Float32MultiArray>("/mainControl/setpoint", 1000);
    ros::Publisher positionPub = rosNH.advertise<std_msgs::Float32MultiArray>("/mainControl/position", 1000);
    ros::Publisher errorPub = rosNH.advertise<std_msgs::Float32MultiArray>("/mainControl/error", 1000);

    // param = which task
    int task = 0;
    if(rosNH.getParam("/t", task) == false)
        rosNH.setParam("/t", task);

    // param = rate
    double Ts = 10;
    if(rosNH.getParam("/Ts", Ts) == false)
        rosNH.setParam("/Ts", Ts);

    // param = lambda
    double lambda = 1;
    if(rosNH.getParam("/lambda", lambda) == false)
        rosNH.setParam("/lambda", lambda);

    // messages to publish
    std_msgs::Float32MultiArray setpointMsg, positionMsg, errorMsg;
    setpointMsg.data.resize(N);
    positionMsg.data.resize(N);
    errorMsg.data.resize(N);

    // points a atteindre
    vpColVector p1(N), p2(N), pe;
    switch(N)
    {
    case 3:
        p1[0] = .065;        p1[1] = -0.02;        p1[2] = .61;
        p2[0] = -.046;        p2[1] = -.046;        p2[2] = .6;
        break;
    case 4:
        p1[0] = .3;        p1[1] = 0.1;        p1[2] = .2;
        p2 = p1;        p1[3] = .4;        p2[3] = -.4;
        break;
    default:break;
    }
    const unsigned int pas = 100; // pas pour generation de trajectoire
    const double vMax = 1; // vitesse maximum dans espace operationnel


    // variables de la boucle
    vpColVector pd = p1;    // destination
    vpColVector p0 = p2;    // point de depart
    unsigned int count = 1;
    const double invPas = 1./(double(pas));
    vpColVector qCmd(N), q(N);
    qCmd = 0;
    unsigned int iter = 0, i;
    vpColVector p(N);
    vpMatrix J(N,N);
    vpColVector v(N), qd(N);
    int tSwitch;

    while(ros::ok())
    {
        iter++;
        // update parameters
        rosNH.getParam("/t", task);
        rosNH.getParam("/Ts", Ts);
        rosNH.getParam("/lambda", lambda);

        // update q and p
        robot.getPosition(q);
        calcMGD(q, p, N);

        // switch p1 <-> p2
        tSwitch = Ts*rosRate;
        if(iter % tSwitch == 0)
        {
            count = 1;
            p0 = pd;
            if(iter % (2*tSwitch) == 0)
                pd = p2;
            else
                pd = p1;
        }
        pe = p - pd;    // erreur courante

        // switch task
        switch(task)
        {
        case 0: // question Q2, affiche juste le MGD et n'envoie pas de commande
            std::cout << "q   : " << q.t() << std::endl;
            std::cout << "MDG : " << p.t() << std::endl;
            break;


        case 1: // oscille entre p1 et p2 dans l'espace articulaire
            calcMGI(q,pd,qd, N);
            robot.setPosition(qd);
            std::cout << "pos error: " << pe.t() << std::endl;
            break;


        case 2: // oscille avec generation de trajectoire
            if(count < pas)
            {
                calcMGI(q, p0 + count*invPas*(pd-p0), qd, N);
                robot.setPosition(qd);
                std::cout << "pos error: " << pe.t() << std::endl;
                count++;
                break;
            }


        case 3: // oscille avec commande en vitesse
            calcJac(q, J, N);
            v = -lambda * pe;                   // vitesse operationnelle
            if(v.euclideanNorm() > vMax)
                v *= vMax/v.euclideanNorm();    // vitesse saturee si superieur au max
            robot.setVelocity(J.pseudoInverse() * v);
            std::cout << "pos error: " << pe.t() << std::endl;
            break;


        default:break;
        }

        // update msgs
        for(i=0;i<N;++i)
        {
            setpointMsg.data[i] = pd[i];
            positionMsg.data[i] = p[i];
            errorMsg.data[i] = pe[i];
        }
        setpointPub.publish(setpointMsg);
        positionPub.publish(positionMsg);
        errorPub.publish(errorMsg);

        ros::spinOnce();
        loop.sleep();


    }

    std::cout << "Stopping node" << std::endl;
}
