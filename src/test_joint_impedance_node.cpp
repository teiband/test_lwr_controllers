#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>
#include <lwr_controllers/PoseRPY.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <test_lwr_controllers/test_controller.hpp>
#include <test_lwr_controllers/motion_generator.hpp>

#include <math.h>
#include <signal.h>

using namespace cv;
using namespace std;

#define LOOP_RATE           1000  // cycles per second to publish new control input
#define NUM_LWR_JOINTS      7

#define SWITCHING_TIME      15 // time between position and impedance controller switch for testing

/// Global Variables

int pos_slider0, stiffness_slider, damping_slider;
double stiffness, damping;

void callbackStiffnessSlider( int, void* )
{
    stiffness = (float)stiffness_slider;
    std::max(0.1, stiffness);
}

void callbackDampingSlider( int, void* )
{
    damping = damping_slider / 100.0;
    std::max(0.01, damping);
}

// global reference to testClass
TestController *global_tester_ptr = NULL;

// this handler is called when you press Ctrl-C to switch back to position control mode
void mySigintHandler(int sig)
{
    global_tester_ptr->switchController("cartesian_position", global_tester_ptr->getControllerName());
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
    ros::init(argc, argv, "test_lwr_controllers");

    ros::NodeHandle nh;

    std::string position_controller("cartesian_position");
    std::string impedance_controller("itr_joint_impedance_controller");

    JointController tester(nh, impedance_controller, position_controller, "right");

    // set global pointer (for signint handler)
    global_tester_ptr = &tester;
    signal(SIGINT, mySigintHandler);

    std_msgs::Float64MultiArray pos;
    pos.data.resize(7);

    std_msgs::Float64MultiArray init_pos;
    init_pos.data.resize(7);

    ros::Rate loop_rate(LOOP_RATE);

    float inc = +0.0002;
    float inc_stiff = +0.1;

    float upper, lower;
    bool initialized = false;

    enum controller_states {JOINT_IMPEDANCE, SWITCH_IMP2POS, JOINT_POSITION, SWITCH_POS2IMP};

    int state = JOINT_IMPEDANCE;
    ros ::Time startTime = ros::Time::now();

    MotionGenerator motion;

    int display_counter;

    while(ros::ok())
	{
        display_counter++;

        switch (state) {

        case JOINT_IMPEDANCE:

            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(SWITCHING_TIME).toSec()) {
                state = SWITCH_IMP2POS;
                ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            if (tester.isInitialized()) {
                if (!initialized) {
                    pos.data = tester.getJointPosition();
                    init_pos.data = tester.getJointPosition();
                    initialized = true;
                    motion.reset();
                }

                pos.data[0] = motion.sineWave(0.3, init_pos.data[0], 0.1, LOOP_RATE);

                if (display_counter%LOOP_RATE == 0) {
                    for (int i=0; i<7; i++) {
                        std::cout << pos.data[i] << ", ";

                    }
                    std::cout << std::endl;
                }

                tester.pub_pose_.publish(pos);
                //tester.pub_force_.publish(torque_vector);
                //tester.pub_gains_.publish(tester.setJointGainsVector(stiffness, damping));

            }
            break;
        case SWITCH_IMP2POS:
        {
            tester.switchController(position_controller, impedance_controller);
            ROS_INFO("Switching to POSITION mode done!");
            state = JOINT_POSITION;

            // reset timer here
            startTime = ros::Time::now();

            motion.reset();

            pos.data = tester.getJointPosition();
            init_pos = pos;

            break;
        }
        case JOINT_POSITION:
            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(SWITCHING_TIME).toSec()) {
                state = SWITCH_POS2IMP;
                startTime = ros::Time::now();
                ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            pos.data[0] = motion.sineWave(0.3, init_pos.data[0], 0.1, LOOP_RATE);

            if (display_counter%LOOP_RATE == 0) {
                for (int i=0; i<7; i++) {
                    std::cout << pos.data[i] << ", ";

                }
                std::cout << std::endl;
            }

            // TODO publish position here to POSITION CONTROLLER

            break;

        case SWITCH_POS2IMP:
        {
            tester.switchController(position_controller, impedance_controller);
            ROS_INFO("Switching to JOINT IMPEDANCE mode done!");
            state = JOINT_IMPEDANCE;

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
