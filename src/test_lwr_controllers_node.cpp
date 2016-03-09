#include <ros/ros.h>
// #include <control_msgs/FollowJointTrajectoryActionGoal.h>
// #include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>

#include <lwr_controllers/PoseRPY.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <test_lwr_controllers/test_controller.h>

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


float swing(float min, float max, double& act, float& inc)
{
    if (act > max || act < min)
        inc = -inc;

    act += inc;
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
    testController tester(nh);

    std::vector<std::string> start_controllers;
    start_controllers.push_back("itr_joint_impedance_controller");
    std::vector<std::string> stop_controllers;
    stop_controllers.push_back("joint_trajectory_controller");
    tester.switchController(start_controllers, stop_controllers);


    std_msgs::Float64MultiArray pos;
    pos.data.resize(7);

    ros::Rate loop_rate(LOOP_RATE);

    float inc = +0.0002;

    float upper, lower;
    bool initialized = false;

    enum controller_states {JOINT_IMPEDANCE, SWITCH_IMP2POS, JOINT_POSITION, SWITCH_POS2IMP};

    int state = JOINT_IMPEDANCE;
    ros ::Time startTime = ros::Time::now();

    while(ros::ok())
	{
        switch (state) {

        case JOINT_IMPEDANCE:

            if ((ros::Time::now().toSec() - startTime.toSec()) >= ros::Time(10).toSec()) {
                state = SWITCH_IMP2POS;
                ROS_INFO("Switched to state SWITCH_IMP2POS");
            }

            if (tester.isInitialized()) {
                if (!initialized) {
                    pos.data = tester.getJointState().position;
                    // set limits:
                    upper = pos.data[0] + 0.2;
                    lower = pos.data[0] - 0.2;
                    initialized = true;
                }

                swing(lower, upper, pos.data[0], inc);

//                pos.data[0] += inc;
//                if (pos.data[0] <= lower) {
//                    inc = -inc;
//                }
//                if (pos.data[0] >= upper) {
//                    inc = -inc;
//                }



                for (int i=0; i<7; i++)
                    std::cout << pos.data[i] << ", ";
                std::cout << std::endl;

                std_msgs::Float64MultiArray torque_vector;
                torque_vector.data.resize(NUM_LWR_JOINTS);
                torque_vector.data[0] = 0;

                swing(-20.0, 20.0, torque_vector.data[0], inc);

                //tester.pub_pos_.publish(pos);
                tester.pub_torque_.publish(torque_vector);
                //tester.pub_gains_.publish(tester.setGainsVector(300.0, 0.7));
                //tester.pub_gains_.publish(tester.setGainsVector(stiffness, damping));

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
