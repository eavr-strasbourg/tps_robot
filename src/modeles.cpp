#include <tps_robot/tps_robot.h>
#include <visp/vpHomogeneousMatrix.h>
using namespace std;

// Fonctions a definir pendant le TP (apres le main() )
// Pour chaque robot: MGD, MGI, Jacobien

// calcul modele geometrique direct
int TPSRobot::calcMGD(const vpColVector &q, vpColVector &pose)
{
    if(name_ == "turret")
    {


    }
    else if(name_ == "scara")
    {


    }
    return 0;
}


// calcul modele geometrique inverse analytique ou iteratif
int TPSRobot::calcMGI(const vpColVector &q0, const vpColVector &pose_des, vpColVector &q)
{
    if(name_ == "turret")
    {


    }
    else if(name_ == "scara")
    {


    }
    return 0;
}


// calcul  Jacobien
int TPSRobot::calcJacobian(const vpColVector &q, vpMatrix &J)
{
    if(name_ == "turret")
    {


    }
    else if(name_ == "scara")
    {


    }
    return 0;
}

