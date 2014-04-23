
#include <tps_robot/tps_robot.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visp/vpHomogeneousMatrix.h>

using namespace std;

// Fonctions a definir pendant le TP (apres le main() )
// Pour chaque robot: MGD, MGI, Jacobien
namespace turret
{
const double b=.5, d=.1;
const unsigned int m = 3;
// MGD 3 ddl
int calcMGD(const vpColVector &q, vpColVector &p);
// Jacobien 3 ddl
int calcJac(const vpColVector &q, vpMatrix &J);
// MGI 3 ddl
int calcMGI(const vpColVector &qi, const vpColVector &p, vpColVector &q);
}
namespace scara {
const double h=.27, a=.2, b=.15, c=.07;
const unsigned int m = 4;
// MGD 4 ddl
int calcMGD(const vpColVector &q, vpColVector &p);
// Jacobien 4 ddl
int calcJac(const vpColVector &q, vpMatrix &J);
// MGI iteratif 4 ddl
int calcMGI(const vpColVector &qi, const vpColVector &pd, vpColVector &q);
}


// Fonctions generiques a utiliser dans le main()
int calcMGD(const vpColVector &q, vpColVector &p, const unsigned int &N)
{
    switch(N)
    {
    case 3: return turret::calcMGD(q, p);break;
    case 4: return scara::calcMGD(q, p);break;
    default:break;
    }
    return -1;
}
int calcJac(const vpColVector &q, vpMatrix &J, const unsigned int &N)
{
    switch(N)
    {
    case 3: return turret::calcJac(q, J);break;
    case 4: return scara::calcJac(q, J);break;
    default:break;
    }
    return -1;
}
int calcMGI(const vpColVector &qi, const vpColVector &pd, vpColVector &q, const unsigned int &N)
{
    switch(N)
    {
    case 3: return turret::calcMGI(qi, pd, q);break;
    case 4: return scara::calcMGI(qi, pd, q);break;
    default:break;
    }
    return -1;
}


// debut du main

int main(int argc, char ** argv)
{
    // initialisation du node ROS
    ros::init(argc, argv, "main_control");
    ros::start();
    ros::NodeHandle rosNH;
    const double rate = 30;
    ros::Rate loop(rate);

    // initialisation de la classe robot
    tpRobot robot(rosNH);
    const unsigned int N = robot.getDOFs();

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

    // *** exemple d'un publisher supplementaire
    // declaration du publisher
    ros::Publisher examplePub = rosNH.advertise<std_msgs::Float32MultiArray>("/example", 1000);
    // declaration du message a publier
    std_msgs::Float32MultiArray example;example.data.resize(3);
    // *** fin example

    // variables utilisees dans la boucle
    unsigned int iter = 0;      // iterations
    vpColVector q(N);           // vecteur des positions articulaires
    vpColVector p_true(N);      // vecteur de la vraie position (X Y Z R P Y)
    vpColVector qCommand(N);    // consigne des positions articulaires
    vpColVector vCommand(N);    // consigne en vitesse

    // points entre lesquels osciller
    vpColVector x1(N), x2(N);
    // definir ici les valeurs des points
    if(N==3)
    {   // x1[0] = ...

    }
    else
    {   // x1[0] = ...

    }

    vpColVector xd = x1, x0 = x2;
    const double T = 10; // temps au bout duquel changer de consigne
    const unsigned int iterSwitch = T*rate;
    
    // boucle de commande
    while(ros::ok())
    {
        // incrementation de l'iteration
        iter++;
        // update parameters
        rosNH.getParam("/t", task);
        rosNH.getParam("/Ts", Ts);
        rosNH.getParam("/lambda", lambda);

        // switch entre x1 et x2
        if(iter % iterSwitch == 0)
        {
            x0 = xd;
            if(iter % (2*iterSwitch) == 0) {xd = x2;}
            else                           {xd = x1;}
        }
        // ici xd est le point a atteindre et x0 est le point de depart

        
        // lecture des positions articulaires
        robot.getPosition(q);
        cout << "Lecture positions articulaires : " << q.t() << endl;

        // *** Modifier ici pour generer une commande en position (qCommand) ou en vitesse (vCommand)

        // switch task
        switch(task)
        {
        case 0: // question Q2, affiche juste le MGD et n'envoie pas de commande

            break;


        case 1: // oscille entre p1 et p2 dans l'espace articulaire

            break;


        case 2: // oscille avec generation de trajectoire

                break;


        case 3: // oscille avec commande en vitesse

            break;


        default:
            cout << "task = " << task << ", rien a faire" << endl;
            break;
        }
        

        // decommenter pour envoyer la commande en position
        //robot.setPosition(qCommand);
        
        // decommenter pour envoyer la commande en vitesse
        //robot.setVelocity(vCommand);
        
        
        
        
        // *** exemple d'un publisher supplementaire
        // mise a jour de exemple.data
        for(unsigned int i=0;i<3;++i)
            example.data[i] = cos(0.1*(i+1)*iter);
        // publication du message
        examplePub.publish(example);
        // *** fin example


        // fin de la boucle
        ros::spinOnce();
        loop.sleep();
    }
}



// fonctions a modifier


// MGD 3 ddl
int turret::calcMGD(const vpColVector &q, vpColVector &p)
{
    return 0;
}

// Jacobien 3 ddl
int turret::calcJac(const vpColVector &q, vpMatrix &J)
{
    return 0;
}

// MGI 3 ddl
int turret::calcMGI(const vpColVector &qi, const vpColVector &p, vpColVector &q)
{
    return 0;
}


// MGD 4 ddl
int scara::calcMGD(const vpColVector &q, vpColVector &p)
{
    return 0;
}

// Jacobien 4 ddl
int scara::calcJac(const vpColVector &q, vpMatrix &J)
{
    return 0;
}

// MGI iteratif 4 ddl
int scara::calcMGI(const vpColVector &qi, const vpColVector &pd, vpColVector &q)
{
    return 0;
}

