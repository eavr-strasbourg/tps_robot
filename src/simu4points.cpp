#include <stdlib.h>
#include <stdio.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpImageConvert.h>
#include <visp/vpMath.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorViper850.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

// create a class to get topics
class rosListener
{
private:
    vpTranslationVector T;

public:
    inline rosListener() {T[2] = 1;}

    void rosReadSetPoint(const std_msgs::Float32MultiArray& msg)
    {
        for(unsigned int i=0;i<2;++i)
            T[i] = msg.data[i];
    }

    inline void getSetPoint(vpTranslationVector &Tr) {Tr = T;}
};


int
main(int argc, char ** argv)
{
    // ROS stuff
    ros::init(argc, argv, "main_control");
    ros::start();
    ros::NodeHandle rosNH;
    const int rosRate = 100;
    ros::Rate loop(rosRate);

    // Subscriber : setpoint
    rosListener listener;
    ros::Subscriber setpointSub = rosNH.subscribe("/setpoint", 1000, &rosListener::rosReadSetPoint, &listener);

    // Publishers : position / error
    ros::Publisher errorPub = rosNH.advertise<std_msgs::Float32MultiArray>("/error", 1000);
    ros::Publisher positionPub = rosNH.advertise<std_msgs::Float32MultiArray>("/position", 1000);
    ros::Publisher imgPub = rosNH.advertise<sensor_msgs::Image>("camera/image", 1000);

    // messages to publish
    std_msgs::Float32MultiArray errorMsg;
    errorMsg.data.resize(3);
    std_msgs::Float32MultiArray positionMsg;
    positionMsg.data.resize(2);

    // image message
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    sensor_msgs::Image imgMsg;

    // get Kp gain
    double Kp;
    if(rosNH.getParam("/Kp", Kp) == false)
        rosNH.setParam("/Kp", .8);

    // get pseudo-inverse damping factor
    double ld;
    if(rosNH.getParam("/ld", ld) == false)
        rosNH.setParam("/ld", .0);

    // ***** SIMULATION CODE *****

    vpDisplayX displayInt;
    // open a display for the visualization
    vpImage<unsigned char> Iint(480, 640, 255);
    displayInt.init(Iint,700,0, "Internal view");
    vpImage<vpRGBa> Irgb(480, 640, 255);

    int i;
    vpServo task;

    // sets the initial camera location
    vpHomogeneousMatrix cMo(-.0,-.0,1,
                            vpMath::rad(10),  vpMath::rad(10),  vpMath::rad(30));

    // sets the point coordinates in the object frame
    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.045,-0.045,0) ;
    point[3].setWorldCoordinates(-0.045,0.045,0) ;
    point[2].setWorldCoordinates(0.045,0.045,0) ;
    point[1].setWorldCoordinates(0.045,-0.045,0) ;

    // computes the point coordinates in the camera frame and its 2D coordinates
    for (i = 0 ; i < 4 ; i++)
        point[i].track(cMo) ;

    // sets the desired position of the point
    vpFeaturePoint p[4], pd[4];
    for (i = 0 ; i < 4 ; i++)
        vpFeatureBuilder::create(p[i],point[i])  ;  //retrieve x,y and Z of the vpPoint structure
    //Desired pose
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0,0.0,0.8,vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)));

    // Projection of the points
    for (int i = 0 ; i < 4 ; i++)
        point[i].track(cdMo);

    for (int i = 0 ; i < 4 ; i++)
        vpFeatureBuilder::create(pd[i], point[i]);

    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT) ;

    // - we want to see a point on a point
    for (i = 0 ; i < 4 ; i++)
        task.addFeature(p[i],pd[i]) ;

    // set the gain
    task.setLambda(Kp) ;

    // Declaration of the robot
    vpSimulatorViper850 robot(true);

    // Augment joint limits
    vpColVector jointMax = robot.getJointMax();
    vpColVector jointMin = robot.getJointMin();
    robot.setJointLimit(jointMin - .5*(jointMax-jointMin), jointMax + .5*(jointMax-jointMin));

    // Initialise the robot and especially the camera
    robot.init(vpViper850::TOOL_PTGREY_FLEA2_CAMERA,vpCameraParameters::perspectiveProjWithoutDistortion);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // Initialise the object for the display part
    robot.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);

    // Initialise the position of the object relative to the pose of the robot's camera
    robot.initialiseObjectRelativeToCamera(cMo);

    // Set the desired position (for the display part)
    robot.setDesiredCameraPosition(cdMo);

    // Get the internal robot's camera parameters
    vpCameraParameters cam(800,800,Iint.getWidth()/2, Iint.getHeight()/2);
    //  robot.getCameraParameters(cam,Iint);


    //Get the internal view
    vpDisplay::display(Iint);
    robot.getInternalView(Iint);
    vpDisplay::flush(Iint);

    unsigned int iter=0 ;
    vpTranslationVector cdTo;
    vpMatrix eJe, I6;I6.eye(6);
    vpVelocityTwistMatrix eWc;
    robot.get_cVe(eWc);
    eWc = eWc.inverse();

    // loop
    while(ros::ok() && (vpDisplay::getClick(Iint, false) == false))
    {
        iter++;
        std::cout << "---------------------------------------------" << iter <<std::endl ;
        vpColVector v ;

        //Get the current pose of the camera
        cMo = robot.get_cMo();

        // update Gain
        rosNH.getParam("/Kp", Kp);
        task.setLambda(Kp) ;

        // update cdMo
        listener.getSetPoint(cdTo);
        cdMo.insert(cdTo);
        robot.setDesiredCameraPosition(cdMo);

        // Projection of the new current and desired points
        for (int i = 0 ; i < 4 ; i++)
        {
            point[i].track(cdMo);
            vpFeatureBuilder::create(pd[i], point[i]);

            point[i].track(cMo) ;
            vpFeatureBuilder::create(p[i],point[i])  ;
        }

        // update only if there is a setpoint
        if(cdMo[0][3] != 0 && cdMo[1][3] != 0)
        {
            // Get the internal view and display it
            vpDisplay::display(Iint) ;
            robot.getInternalView(Iint);
            vpDisplay::flush(Iint);

            // compute the control law
            v = task.computeControlLaw();

            robot.get_eJe(eJe);
            rosNH.getParam("/ld", ld);

            // send the camera velocity to the controller
            robot.setVelocity(vpRobot::ARTICULAR_FRAME, (eJe + ld*I6).pseudoInverse() * eWc * v);
        }

        // ***** ROS - update and publish messages *****

        // position
        for(unsigned int i=0;i<2;++i)
            positionMsg.data[i] = cMo[i][3];
        positionPub.publish(positionMsg);

        // error
        for(unsigned int i=0;i<3;++i)
            errorMsg.data[i] = cdMo[i][3] - cMo[i][3];
        errorPub.publish(errorMsg);

        // image
        displayInt.getImage(Irgb);
        vpImageConvert::convert(Irgb, cvi.image);
        cvi.header.stamp = ros::Time::now();
        cvi.toImageMsg(imgMsg);

        imgPub.publish(imgMsg);


        // fin de la boucle
        ros::spinOnce();
        loop.sleep();
    }
    task.kill();
}
