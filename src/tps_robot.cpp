
#include <tpRobots/tpRobot.h>
#include <urdf/model.h>
#include <algorithm>

using std::string;
using std::cout;
using std::endl;

tpRobot::tpRobot(ros::NodeHandle &mainNode)
{    
    // parse URDF to get robot data (name, DOF, joint limits, etc.)
    urdf::Model model;
    model.initParam("/robot_description");

    // DOF's and maximum velocity for each
    vMax.clear();
    boost::shared_ptr<urdf::Joint> joint;
    n = 0;
    for (std::map<std::string,boost::shared_ptr<urdf::Joint> >::iterator joint_it = model.joints_.begin();joint_it != model.joints_.end(); joint_it++)
    {
        joint = joint_it->second;
        if(joint->type != urdf::Joint::FIXED)
        {
            n += 1;
            vMax.push_back(joint->limits->velocity);
        }
    }
    cout << "Found robot description: " << model.getName() << " with " << n << " DOFs" << endl;



    // initialise inner variables
    q.resize(n);
    tfP.resize(6);
    command.position.resize(n);
    command.velocity.resize(n);
    command.name.resize(n);

    char qi[FILENAME_MAX];
    unsigned int i;
    for(i=0;i<n;i++)
    {
        sprintf(qi, "%d", i+1);
        command.name[i] = "joint" + std::string(qi);
    }

    // initialise ROS topics: publisher for command, subscriber for position measurement
    commandPub = mainNode.advertise<sensor_msgs::JointState>("/mainControl/command", 1000);
    positionSub = mainNode.subscribe("/joint_states", 1000, &tpRobot::rosReadPosition, this);
    tfListener = NULL;
    ros::spinOnce();
}

// initialize tf listener and base and end links
void tpRobot::initTF(tf::TransformListener &listener, const std::string & base_link, const std::string & end_effector)
{
    tfListener = &listener;
    tfBase = base_link;
    tfEnd = end_effector;
}

// read joint_state topic and write articular position
void tpRobot::rosReadPosition(const sensor_msgs::JointState::ConstPtr& msg)
{
    // parse message to joint position, check for joint names
    for(unsigned int i=0;i<n;++i)
        for(unsigned int j=0;j<n;++j)
        {
            if(msg->name[j] == command.name[i])
                q[i] = msg->position[j];
        }

    // also read tf
    if(tfListener != NULL)
    {
        try{
            tfListener->lookupTransform(tfBase, tfEnd, ros::Time(0), transform);
            tfP[0] = transform.getOrigin().x();
            tfP[1] = transform.getOrigin().y();
            tfP[2] = transform.getOrigin().z();
            Rq.set(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
            tfR.buildFrom(Rq);
            Rxyz.buildFrom(tfR);
            tfP[3] = Rxyz[0];
            tfP[4] = Rxyz[1];
            tfP[5] = Rxyz[2];
        }
        catch (tf::TransformException ex){    }
    }

    // if(rosVerbose)
    //     std::cout << "tpRobot - Read position from robot : " << q.t() << std::endl;
}


// set articular position
void tpRobot::setPosition(const vpColVector &position)
{
    if(position.getRows() != n)
        std::cout << "tpRobot::setPosition: bad dimension" << std::endl;
    else
    {
        command.header.stamp = ros::Time::now();
        for(unsigned int i=0;i<n;++i)
        {
            command.velocity[i] = 0;
            command.position[i] = position[i];
        }

        commandPub.publish(command);
    }
}


// set articular velocity
void tpRobot::setVelocity(vpColVector velocity)
{
    if(velocity.getRows() != n)
        std::cout << "tpRobot::setVelocity: bad dimension" << std::endl;
    else
    {
        command.header.stamp = ros::Time::now();

        // if vMax, scales velocity (educational goal)
        unsigned int i;
        double scale = 1.;
        if(vMax.size()!=0)
        {
            for(i=0;i<n;++i)
                scale = std::max(scale, std::abs(velocity[i])/vMax[i]);
            if(scale > 1)
                std::cout << "scaling v: " << scale << std::endl;
            scale = 1./scale;

        }

        for(i=0;i<n;++i)
            command.velocity[i] = velocity[i] * scale;

        commandPub.publish(command);
    }
}
