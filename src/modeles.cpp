#include <tps_robot/tps_robot.h>
#include <visp/vpHomogeneousMatrix.h>
using namespace std;

// Fonctions a definir pendant le TP (apres le main() )
// Pour chaque robot: MGD, MGI, Jacobien

// calcul modele geometrique direct
int Robot::calcMGD(const vpColVector &q, vpColVector &pose)
{
    if(robot_name_ == "turret")
    {


    }
    else if(robot_name_ == "scara")
    {


    }
    return 0;
}


// calcul modele geometrique inverse analytique ou iteratif
int Robot::calcMGI(const vpColVector &q0, const vpColVector &pose_des, vpColVector &q)
{
    if(robot_name_ == "turret")
    {


    }
    else if(robot_name_ == "scara")
    {


    }
    return 0;
}


// calcul  Jacobien
int Robot::calcJacobian(const vpColVector &q, vpMatrix &J)
{
    if(robot_name_ == "turret")
    {


    }
    else if(robot_name_ == "scara")
    {


    }
    return 0;
}

