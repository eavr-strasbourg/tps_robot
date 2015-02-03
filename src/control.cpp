
#include <tps_robot/tps_robot.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using TPS::Robot;

int main(int argc, char ** argv)
{
    // initialisation du node ROS
    ros::init(argc, argv, "main_control");
    ros::start();
    ros::NodeHandle rosNH;
    const double rate = 30;
    ros::Rate loop(rate);

    // initialisation de la classe robot
    Robot robot(rosNH);
    const unsigned int N = robot.getDOFs();

    // Tache a realiser, defaut 0 (affiche seulement)
    int question = 0;
    rosNH.param("/Q", question, 0);

    // Temps au bout duquel changer de consigne, defaut 10 s
    double Ts;
    rosNH.param("/Ts", Ts, 10.);

    // Gain du controle en vitesse, defaut 1
    double lambda;
    rosNH.param("/lambda", lambda, 1.);

    // *** exemple d'un publisher supplementaire : erreur de position
    // declaration du publisher
    ros::Publisher erreurPub = rosNH.advertise<std_msgs::Float32MultiArray>("/erreur", 1000);
    // declaration du message a publier
    std_msgs::Float32MultiArray erreur;erreur.data.resize(3);
    // *** fin example

    // variables utilisees dans la boucle
    unsigned int iter = 0;      // iterations
    vpColVector q(N);           // vecteur des positions articulaires
    vpColVector p(N);           // vecteur de la vraie position (X Y Z R P Y)
    vpColVector qCommand(N);    // consigne des positions articulaires
    vpColVector vCommand(N);    // consigne en vitesse

    // points entre lesquels osciller
    vpColVector p1(N), p2(N);
    // definir ici les valeurs des points
    switch(N)
    {
    case 3:

        break;
    case 4:

        break;
    default:

    }
    const unsigned int pas = 100; // pas pour generation de trajectoire
    const double invPas = 1./pas;
    const double vMax = 1; // vitesse maximum dans espace operationnel


    vpColVector pd = p1, p0 = p2;
    unsigned int iterSwitch;
    vpMatrix J;
    unsigned int avance;
    
    // boucle de commande
    while(ros::ok())
    {
        // incrementation de l'iteration
        iter++;
        // update parameters
        rosNH.getParam("/Q", question);
        rosNH.getParam("/Ts", Ts);
        rosNH.getParam("/lambda", lambda);
        iterSwitch = Ts*rate;

        // switch entre p1 et p2
        if(iter % iterSwitch == 0)
        {
            p0 = pd;
            if(iter % (2*iterSwitch) == 0) {pd = p2;}
            else                           {pd = p1;}
        }
        // pd est le point a atteindre et p0 est le point de depart

        
        // lecture des positions articulaires
        robot.getPosition(q);
        cout << "Lecture positions articulaires : " << q.t() << endl;


        // calcul MGD
        robot.compDK(q, p);


        // *** Modifier ici pour generer une commande en position (qCommand) ou en vitesse (vCommand)

        // switch question
        switch(question)
        {
        case 0: // question Q2, affiche juste le MGD et n'envoie pas de commande

            break;

        case 1: // oscille entre p1 et p2 dans l'espace articulaire

            // decommenter pour envoyer la commande en position
            //robot.setPosition(qCommand);
            break;
            
        case 2: // oscille avec generation de trajectoire

            // decommenter pour envoyer la commande en position
            //robot.setPosition(qCommand);
            break;
            
        case 3: // oscille avec commande en vitesse

            // decommenter pour envoyer la commande en vitesse
            //robot.setVelocity(vCommand);
            break;

        default:
            cout << "question = " << question << ", rien a faire" << endl;
        }
        
        
        
        
        
        // *** exemple d'un publisher supplementaire
        // mise a jour de exemple.data
        for(unsigned int i=0;i<3;++i)
            erreur.data[i] = p[i] - pd[i];
        // publication du message
        erreurPub.publish(erreur);
        // *** fin example

        // fin de la boucle
        ros::spinOnce();
        loop.sleep();
    }
}
