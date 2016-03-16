#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Pose.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>

#include <lwr_controllers/PoseRPY.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>

#include <test_lwr_controllers/test_controller.hpp>
#include <test_lwr_controllers/motion_generator.hpp>

#define DISPLAY_CYCLES  1000
#define SHOW(i) if(i%DISPLAY_CYCLES==0) std::cout <<

using namespace cv;
using namespace std;

#define LOOP_RATE           1000  // cycles per second to publish new control input

/// Global Variables

int pos_slider0, pos_stiffness_slider, orientation_stiffness_slider,damping_slider;
float pos_stiffness, orientation_stiffness, damping;

void callbackPosStiffnessSlider( int, void* )
{
    pos_stiffness = (float)pos_stiffness_slider;
}

void callbackOrientationStiffnessSlider( int, void* )
{
    orientation_stiffness = (float)orientation_stiffness_slider;
}

void callbackDampingSlider( int, void* )
{
    damping = damping_slider / 100.0;
}

// global reference to testClass
TestController *global_tester_ptr = NULL;

// this handler is called when you press Ctrl-C to switch back to position control mode
void mySigintHandler(int sig)
{
    std::vector<std::string> start_controllers;
    start_controllers.push_back("cartesian_position");
    std::vector<std::string> stop_controllers;
    stop_controllers.push_back(global_tester_ptr->getControllerName());
    global_tester_ptr->switchController(start_controllers, stop_controllers);
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char** argv)
{
    // ----- GUI -----
    // Initialize values
    pos_slider0 = 0;
    pos_stiffness_slider = 800;
    orientation_stiffness_slider = 50;
    damping_slider = 70;

    // Create Windows
    namedWindow("SimpleControl", 1);

    // Create Trackbars
    //createTrackbar( "position joint 0", "SimpleControl", &pos_slider0, 360, callbackPosSlider );
    createTrackbar( "pos. stiffness", "SimpleControl", &pos_stiffness_slider, 1500, callbackPosStiffnessSlider );
    createTrackbar( "orientation stiffness", "SimpleControl", &orientation_stiffness_slider, 1500, callbackOrientationStiffnessSlider );
    createTrackbar( "damping", "SimpleControl", &damping_slider, 100, callbackDampingSlider );

    // put text
    cv::Mat pic = cv::Mat::zeros(250,250,CV_8UC3);
    cv::putText(pic, "Press any key after settings!",cv::Point(5,15), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(255),1,8,false);
    cv::imshow("SimpleControl", pic);

    // Show some stuff
    callbackPosStiffnessSlider ( pos_stiffness_slider, 0 );
    callbackOrientationStiffnessSlider ( orientation_stiffness_slider, 0 );
    callbackDampingSlider ( damping_slider, 0 );

    // shows the window until a key is pressed
    waitKey(0);
    // ----- end GUI -----

    // Init the ROS node
    ros::init(argc, argv, "test_lwr_controllers");

    ros::NodeHandle nh;

    std::string position_controller("cartesian_position");
    std::string impedance_controller("itr_cartesian_impedance_controller");

    CartController tester(nh, impedance_controller, position_controller);

    // set global pointer (for signint handler)
    global_tester_ptr = &tester;
    signal(SIGINT, mySigintHandler);

    // lwr_controllers::PoseRPY pose;
    geometry_msgs::Pose pose, pose_robot, pose_world;

    ros::Rate loop_rate(LOOP_RATE);

    float inc = +0.00005;

    float upper, lower;
    bool initialized = false;

    enum controller_states {CARTESIAN_IMPEDANCE, SWITCH_IMP2POS, JOINT_POSITION, SWITCH_POS2IMP};

    int state = CARTESIAN_IMPEDANCE;
    ros ::Time startTime = ros::Time::now();

    MotionGenerator motion;

    int display_counter;

    while(ros::ok())
	{
        display_counter++;

        switch (state) {

        case CARTESIAN_IMPEDANCE:

            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(20).toSec()) {
                state = SWITCH_IMP2POS;
                ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            if (tester.isInitialized()) {
                if (!initialized) {
                    cout << "Initializing main loop ..." << endl;
                    pose_world = tester.getWorldPosition().pose;
                    pose_robot = tester.getCartPosition().pose;
                    // define here, in which coordinate system you want to work
                    pose = pose_world;
                    cout << "init pose: " << pose_world << endl;
                    // set limits:
                    upper = pose.position.z + 0.1;
                    lower = pose.position.z - 0.1;
                    initialized = true;
                }

                // move the robot in one axis ...
                motion.swing(lower, upper, pose.position.z, inc);

                // ROS_INFO_STREAM("Publishing to" << pub_pose.getNumSubscribers() << " subscribers\n");

                tester.pub_pose_world_.publish(pose);

                SHOW(display_counter) "Cart. Position: " << endl << pose << endl;

                tester.pub_gains_.publish(tester.setCartGainsVector(pos_stiffness, orientation_stiffness, damping));
            }
            break;
        case SWITCH_IMP2POS:
        {
            std::vector<std::string> start_controllers;
            start_controllers.push_back(position_controller);
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back(impedance_controller);
            tester.switchController(start_controllers, stop_controllers);
            ROS_INFO("Switching to POSITION mode done!");
            state = JOINT_POSITION;

            // reset timer here
            startTime = ros::Time::now();
            break;
        }
        case JOINT_POSITION:
            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(8).toSec()) {
                //exit(0);
                state = SWITCH_POS2IMP;
                startTime = ros::Time::now();
                ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            break;

        case SWITCH_POS2IMP:
        {
            std::vector<std::string> start_controllers;
            start_controllers.push_back(impedance_controller);
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back(position_controller);
            tester.switchController(start_controllers, stop_controllers);
            ROS_INFO("Switching to JOINT IMPEDANCE mode done!");
            state = CARTESIAN_IMPEDANCE;

            // reset timer here
            startTime = ros::Time::now();

            // reset initialized state for impedance controller
            initialized = false;
            break;
        }
        } // end switch statement

        ros::spinOnce();
        loop_rate.sleep();

	}

}
