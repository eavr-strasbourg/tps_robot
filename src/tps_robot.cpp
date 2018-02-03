
#include <tps_robot/tps_robot.h>
#include <urdf/model.h>
#include <algorithm>

using std::string;
using std::cout;
using std::endl;
using TPS::Robot;

Robot::Robot(ros::NodeHandle &_nh)
{    
    // parse URDF to get robot data (name, DOF, joint limits, etc.)
    urdf::Model model;
    model.initParam("/robot_description");

    // store name
    name_ = model.getName();

    // init joints
    v_max_.clear();
    urdf::JointSharedPtr joint;
    n_ = 0;
    cmd_.name.clear();
    for (std::map<std::string,urdf::JointSharedPtr >::iterator joint_it = model.joints_.begin();joint_it != model.joints_.end(); joint_it++)
    {
        joint = joint_it->second;
        if(joint->type != urdf::Joint::FIXED)
        {
            n_ += 1;
            v_max_.push_back(joint->limits->velocity);
            cmd_.name.push_back(joint->name);
        }
    }
    cout << "Found robot description: " << model.getName() << " with " << n_ << " DOFs" << endl;

    q_.resize(n_);
    cmd_.position.resize(n_);
    cmd_.velocity.resize(n_);

    // initialise ROS topics: publisher for command, subscriber for position measurement
    cmd_publisher_ = _nh.advertise<sensor_msgs::JointState>("/main_control/command", 1000);
    position_subscriber_ = _nh.subscribe("/joint_states", 1000, &Robot::onReadPosition, this);
    ros::spinOnce();
}

// read joint_state topic and write articular position
void Robot::onReadPosition(const sensor_msgs::JointState::ConstPtr& _msg)
{
    // parse message to joint position, check for joint names
    for(unsigned int i=0;i<n_;++i)
        for(unsigned int j=0;j<n_;++j)
        {
            if(_msg->name[j] == cmd_.name[i])
                q_[i] = _msg->position[j];
        }
}


// set articular position
void Robot::setPosition(const vpColVector &_position)
{
    if(_position.getRows() != n_)
        std::cout << "Robot::setPosition: bad dimension" << std::endl;
    else
    {
        cmd_.header.stamp = ros::Time::now();
        for(unsigned int i=0;i<n_;++i)
        {
            cmd_.velocity[i] = 0;
            cmd_.position[i] = _position[i];
        }

        cmd_publisher_.publish(cmd_);
    }
}

// set articular velocity
void Robot::setVelocity(const vpColVector &_velocity)
{
    if(_velocity.getRows() != n_)
        std::cout << "Robot::setVelocity: bad dimension" << std::endl;
    else
    {
        cmd_.header.stamp = ros::Time::now();

        // if v_max_, scales velocity (educational goal)
        unsigned int i;
        double scale = 1.;
        if(v_max_.size()!=0)
        {
            for(i=0;i<n_;++i)
                scale = std::max(scale, std::abs(_velocity[i])/v_max_[i]);
            if(scale > 1)
                std::cout << "scaling v: " << scale << std::endl;
            scale = 1./scale;

        }

        for(i=0;i<n_;++i)
            cmd_.velocity[i] = _velocity[i] * scale;

        cmd_publisher_.publish(cmd_);
    }
}

