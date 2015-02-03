#ifndef TPSROBOT_H
#define TPSROBOT_H

#include <visp/vpColVector.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpRxyzVector.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

/*
  Classe de robot générique
  Reçoit des commandes en vitesse / position et les envoie sur le bridge avant interprétation par le simulateur.
*/

namespace TPS
{

class Robot {

public:
    // --- These methods are defined in tps_robot.cpp ---
    // constructor
    Robot(ros::NodeHandle &_nh);
    // get number of dof
    inline unsigned int getDOFs() {return n_;}
    // get name
    inline std::string getName() {return name_;}
    // get articular position
    inline void getPosition(vpColVector &_position) {_position = q_;}
    // set articular position
    void setPosition(const vpColVector &_position);
    // set articular velocity
    void setVelocity(const vpColVector &_velocity);
    // stop motion
    inline void stopMotion() {setPosition(q_);}
    // --- End defined in tps_robot.cpp ---


    // --- These methods are defined in modeles.cpp ---
    // To be completed by the students

    // calcul modele geometrique direct
    int compDK(const vpColVector &q, vpColVector &pose);

    // calcul modele geometrique inverse analytique ou iteratif
    int compIK(const vpColVector &q0, const vpColVector &pose_des, vpColVector &q);

    // calcul  Jacobien
    int compJacobian(const vpColVector &q, vpMatrix &J);

protected:
    unsigned int n_;
    std::string name_;
    vpColVector q_;
    ros::Publisher cmd_publisher_;
    ros::Subscriber position_subscriber_;
    sensor_msgs::JointState cmd_;

    // maximum velocity
    std::vector<double> v_max_;

    // ROS functions
    void onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg);
};

}



#endif // TPSROBOT_H
