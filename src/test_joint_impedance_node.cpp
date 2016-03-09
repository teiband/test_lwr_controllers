#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>
#include <lwr_controllers/PoseRPY.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <test_lwr_controllers/test_controller.h>
#include <math.h>
#include <signal.h>

using namespace cv;

#define LOOP_RATE           1000  // cycles per second to publish new control input
#define NUM_LWR_JOINTS      7

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


void swing(float min, float max, double& act, float& inc)
{
    if (act > max || act < min)
        inc = -inc;

    act += inc;
}

void sineWave(float amp, double init_pos, double& act, double& time)
{
    const float f = 0.05; //Hz
    act = init_pos + amp * sin(2.0*M_PI* f * time);
    time += 1/(float)LOOP_RATE;
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
    jointController tester(nh, "itr_joint_impedance_controller");

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

    double d_stiffness = stiffness;

    float upper, lower;
    bool initialized = false;

    enum controller_states {JOINT_IMPEDANCE, SWITCH_IMP2POS, JOINT_POSITION, SWITCH_POS2IMP};

    int state = JOINT_IMPEDANCE;
    ros ::Time startTime = ros::Time::now();

    int display_counter;
    double time = 0;

    while(ros::ok())
	{
        display_counter++;

        switch (state) {

        case JOINT_IMPEDANCE:

            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(10).toSec()) {
                //state = SWITCH_IMP2POS;
                //ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            if (tester.isInitialized()) {
                if (!initialized) {
                    pos.data = tester.getJointState().position;
                    init_pos.data = tester.getJointState().position;
                    // set limits:
                    upper = pos.data[0] + 0.3;
                    lower = pos.data[0] - 0.3;
                    initialized = true;
                }

                //swing(lower, upper, pos.data[0], inc);
                sineWave(0.3, init_pos.data[0], pos.data[0], time);

                if (display_counter%LOOP_RATE == 0) {
                    for (int i=0; i<7; i++) {
                        std::cout << pos.data[i] << ", ";

                    }
                    std::cout << std::endl;
                }

                sineWave(550.0, 600.0, d_stiffness, time);

                //swing(200.0, 1500.0, d_stiffness, inc_stiff);

                //std_msgs::Float64MultiArray torque_vector;
                //torque_vector.data.resize(NUM_LWR_JOINTS);
                //torque_vector.data[0] = 0;

                tester.pub_pose_.publish(pos);
                //tester.pub_force_.publish(torque_vector);
                //tester.pub_gains_.publish(tester.setGainsVector(300.0, 0.7));
                tester.pub_gains_.publish(tester.setJointGainsVector(d_stiffness, damping));

            }
            break;
        case SWITCH_IMP2POS:
        {
            std::vector<std::string> start_controllers;
            start_controllers.push_back("cartesian_position");
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("itr_joint_impedance_controller");
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
            start_controllers.push_back("itr_joint_impedance_controller");
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("cartesian_position");
            tester.switchController(start_controllers, stop_controllers);
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
