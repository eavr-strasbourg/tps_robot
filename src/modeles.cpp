#include <tps_robot/tps_robot.h>
#include <visp/vpHomogeneousMatrix.h>

// Fonctions a definir pendant le TP
// Pour chaque robot: MGD, MGI, Jacobien

using namespace std;
using TPS::Robot;

// calcul modele geometrique direct
int Robot::compDK(const vpColVector &q, vpColVector &pose)
{
    if(name_ == "turret")
    {
      

    }
    else if(name_ == "scara")
    {
      

    }
    return 0;
}


// calcul modele geometrique inverse analytique
int Robot::compIK(const vpColVector &q0, const vpColVector &pose_des, vpColVector &q)
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
int Robot::compJacobian(const vpColVector &q, vpMatrix &J)
{
    if(name_ == "turret")
    {
     
    }
    else if(name_ == "scara")
    {
      
    }
    return 0;
}


