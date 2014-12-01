
#include <tps_robot/tps_robot.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visp/vpHomogeneousMatrix.h>

using namespace std;

// debut du main

int main(int argc, char ** argv)
{
    // initialisation du node ROS
    ros::init(argc, argv, "main_control");
    ros::start();
    ros::NodeHandle nh;
    const double rate = 30;
    ros::Rate loop(rate);

    // initialisation de la classe robot
    TPSRobot robot(nh);
    const unsigned int N = robot.getDOFs();
    const std::string name = robot.getName();

    // initialisation des parametres
    // quelle question est a realiser
    int question;
    nh.param<int>("/Q", question, 0);

    // frequence consigne
    double Ts;
    nh.param<double>("/Ts", Ts, 10);

    // gain du controle en vitesse
    double lambda;
    nh.param<double>("/lambda", lambda, 1);
    // fin initialisation des parametres

    // *** exemple d'un publisher supplementaire pour l'erreur de position
    // declaration du publisher
    ros::Publisher error_publisher = nh.advertise<std_msgs::Float32MultiArray>("/error", 1000);
    // declaration du message a publier
    std_msgs::Float32MultiArray error;error.data.resize(3);
    // *** fin example

    // variables utilisees dans la boucle
    unsigned int iter = 0;      // iterations
    vpColVector q(N);           // vecteur des positions articulaires
    vpColVector p(6);           // vecteur de la position operationelle
    vpColVector qCommand(N);    // consigne des positions articulaires
    vpColVector vCommand(N);    // consigne en vitesse

    // points entre lesquels osciller
    vpColVector p1(N), p2(N);
    // definir ici les valeurs des points a atteindre
    if(name == "turret")
    {   // p1[0] = ...

    }
    else
    {   // p1[0] = ...

    }

    vpColVector pd = p1, p0 = p2;
    unsigned int iterSwitch;
    
    // boucle de commande
    while(ros::ok())
    {
        // incrementation de l'iteration
        iter++;
        // update parameters
        nh.getParam("/Q", question);
        nh.getParam("/Ts", Ts);
        nh.getParam("/lambda", lambda);

        // switch entre p1 et p2 en fonction de la frequence
        iterSwitch = Ts*rate;
        if(iter % iterSwitch == 0)
        {
            p0 = pd;
            if(iter % (2*iterSwitch) == 0) {pd = p2;}
            else                           {pd = p1;}
        }
        // a partir d'ici pd est le point a atteindre et p0 est le point de depart

        
        // lecture des positions articulaires
        robot.getPosition(q);
        cout << "Lecture positions articulaires : " << q.t() << endl;

        // *** Modifier ici pour generer une commande en position (qCommand) ou en vitesse (vCommand)

        // differents cas en fonction de la question
        switch(question)
        {
        case 1: // question Q1, affiche juste le MGD et n'envoie pas de commande

            break;


        case 4: // question Q4, oscille entre p1 et p2 dans l'espace articulaire

            break;


        case 8: // question Q8, oscille avec generation de trajectoire

                break;


        case 9: // question Q9, oscille avec commande en vitesse

            break;


        default:
            cout << "question = " << question << ", rien a faire" << endl;
            break;
        }
        

        // decommenter pour envoyer la commande en position
        //robot.setPosition(qCommand);
        
        // decommenter pour envoyer la commande en vitesse
        //robot.setVelocity(vCommand);
        
        
        
        
        // *** exemple d'un publisher supplementaire
        // mise a jour de error.data
        for(unsigned int i=0;i<3;++i)
            error.data[i] = p[i] - pd[i];
        // publication du message
        error_publisher.publish(error);
        // *** fin example


        // fin de la boucle
        ros::spinOnce();
        loop.sleep();
    }
}


