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

#include <test_lwr_controllers/test_controller.h>
#define DISPLAY_CYCLES  1000
#define SHOW(i) if(i%DISPLAY_CYCLES==0) std::cout <<

using namespace cv;
using namespace std;

#define LOOP_RATE           1000  // cycles per second to publish new control input

/// Global Variables

int pos_slider0, stiffness_slider, damping_slider;
float stiffness, damping;

void callbackStiffnessSlider( int, void* )
{
    stiffness = (float)stiffness_slider;
}

void callbackDampingSlider( int, void* )
{
    damping = damping_slider / 100.0;
}


float swing(float min, float max, double& act, float& inc)
{
    if (act > max || act < min)
        inc = -inc;

    act += inc;
}


// global reference to testClass
testController *global_tester_ptr = NULL;

// this handler is called when you press Ctrl-C to switch back to position control mode
void mySigintHandler(int sig)
{
    std::vector<std::string> start_controllers;
    start_controllers.push_back("joint_trajectory_controller");
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
    stiffness_slider = 300;
    damping_slider = 70;

    // Create Windows
    namedWindow("SimpleControl", 1);

    // Create Trackbars
    //createTrackbar( "position joint 0", "SimpleControl", &pos_slider0, 360, callbackPosSlider );
    createTrackbar( "stiffness", "SimpleControl", &stiffness_slider, 1500, callbackStiffnessSlider );
    createTrackbar( "damping", "SimpleControl", &damping_slider, 100, callbackDampingSlider );

    // put text
    cv::Mat pic = cv::Mat::zeros(250,250,CV_8UC3);
    cv::putText(pic, "Press any key after settings!",cv::Point(5,15), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(255),1,8,false);
    cv::imshow("SimpleControl", pic);

    // Show some stuff
    callbackStiffnessSlider ( stiffness_slider, 0 );
    callbackDampingSlider ( damping_slider, 0 );

    // shows the window until a key is pressed
    waitKey(0);
    // ----- end GUI -----

    // Init the ROS node
    ros::init(argc, argv, "test_lwr_controllers_node");

    ros::NodeHandle nh;
    cartController tester(nh, "cartesian_impedance_controller");

    // set global pointer (for signint handler)
    global_tester_ptr = &tester;
    signal(SIGINT, mySigintHandler);

    // lwr_controllers::PoseRPY pose;
    geometry_msgs::Pose pose;

    ros::Rate loop_rate(LOOP_RATE);

    float inc = +0.00005;

    float upper, lower;
    bool initialized = false;

    enum controller_states {CARTESIAN_IMPEDANCE, SWITCH_IMP2POS, JOINT_POSITION, SWITCH_POS2IMP};

    int state = CARTESIAN_IMPEDANCE;
    ros ::Time startTime = ros::Time::now();

    int display_counter;

    while(ros::ok())
	{
        display_counter++;

        switch (state) {

        case CARTESIAN_IMPEDANCE:

            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(20).toSec()) {
                //state = SWITCH_IMP2POS;
                //ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            if (tester.isInitialized()) {
                if (!initialized) {
                    cout << "Initializing main loop ..." << endl;
                    pose = tester.getCartPosition().pose;
                    // set limits:
                    upper = pose.position.z + 0.1;
                    lower = pose.position.z - 0.1;
                    initialized = true;
                }

                swing(lower, upper, pose.position.z, inc);

                // ROS_INFO_STREAM("Publishing to" << pub_pose.getNumSubscribers() << " subscribers\n");

                //pose = tester.getCartPosition().pose;
                tester.pub_pose_.publish(pose);

                SHOW(display_counter) "Cart. Position: " << pose.position.x << ", "
                                                         << pose.position.y << ", "
                                                         << pose.position.z << ", "
                                                         << pose.orientation.x << ", "
                                                         << pose.orientation.y << ", "
                                                         << pose.orientation.z << ", "
                                                         << pose.orientation.w << endl;

                //tester.pub_torque_.publish(torque_vector);
                //tester.pub_gains_.publish(tester.setCartGainsVector(300.0, 0.7));
                //tester.pub_gains_.publish(tester.setCartGainsVector(stiffness, damping));
                std_msgs::Float64MultiArray gains = tester.setCartGainsVector(300, 0.7);
                gains.data[0] = 300.0;
                gains.data[1] = 300.0;
                gains.data[2] = 300.0;
                gains.data[3] = 50.0;
                gains.data[4] = 50.0;
                gains.data[5] = 50.0;
                tester.pub_gains_.publish(gains);
            }
            break;
        case SWITCH_IMP2POS:
        {
            std::vector<std::string> start_controllers;
            start_controllers.push_back("cartesian_position");
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("cartesian_impedance_controller");
            tester.switchController(start_controllers, stop_controllers);
            ROS_INFO("Switching to POSITION mode done!");
            state = JOINT_POSITION;

            // reset timer here
            startTime = ros::Time::now();
            break;
        }
        case JOINT_POSITION:
            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(5).toSec()) {
                exit(0);
                //state = SWITCH_POS2IMP;
                //startTime = ros::Time::now();
                //ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            break;

        case SWITCH_POS2IMP:
        {
            std::vector<std::string> start_controllers;
            start_controllers.push_back("cartesian_impedance_controller");
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("cartesian_position");
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
