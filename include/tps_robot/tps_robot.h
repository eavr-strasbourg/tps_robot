#ifndef TPROBOT_H
#define TPROBOT_H

#include <visp/vpColVector.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpRxyzVector.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <string>

/*
  Classe de robot générique
  Reçoit des commandes en vitesse / position et les envoie sur le bridge avant interprétation par le simulateur.
*/


class tpRobot {

public:
    // constructor
    tpRobot(ros::NodeHandle &mainNode);

    // get number of dof
    inline unsigned int getDOFs() {return n;}

    // get articular position
    inline void getPosition(vpColVector &position) {position = q;}

    // set articular position
    void setPosition(const vpColVector &position);

    // set articular velocity
    void setVelocity(vpColVector velocity);

    // get true transform
    void initTF(tf::TransformListener &listener, const std::string & base_link = "base_link", const std::string & end_effector = "end_effector");
    inline void getTF(vpColVector& position) {position = tfP;}

    // stop motion
    inline void stopMotion() {setPosition(q);}

protected:
    unsigned int n;
    vpColVector q;
    ros::Publisher commandPub;
    ros::Subscriber positionSub;
    sensor_msgs::JointState command;

    // maximum velocity
    std::vector<double> vMax;

    // related to true tf transform
    std::string tfBase, tfEnd;
    vpColVector tfP;
    vpRotationMatrix tfR;
    vpQuaternionVector Rq;
    vpRxyzVector Rxyz;
    tf::TransformListener* tfListener;
    tf::StampedTransform transform;

    ros::NodeHandle* rosNH;

    // ROS functions
    void rosReadPosition(const sensor_msgs::JointState::ConstPtr& msg);
};



#endif // TPROBOT_H
