#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>

#include <lwr_controllers/PoseRPY.h>

#define NUM_LWR_JOINTS      7
#define NUM_CART_DIM        6

using namespace std;

class testController
{
protected:
	
    sensor_msgs::JointState js_;
    bool initialized_;
    lwr_controllers::PoseRPY cart_pose_;
    ros::Subscriber sub_js_;
    controller_manager_msgs::SwitchController sc_;
    controller_manager_msgs::ListControllers lc_;
    ros::ServiceClient srv_contr_switcher_;
    ros::ServiceClient srv_contr_list_;
    std::string controller_name_;
    ros::NodeHandle nh_;

public:

    ros::Publisher pub_pose_;
    ros::Publisher pub_force_;
    ros::Publisher pub_gains_;

    //testController() {}

    testController(ros::NodeHandle &nh, std::string controller_name) : nh_(nh), controller_name_(controller_name)
    { }

    virtual void callbackJointState(const sensor_msgs::JointStateConstPtr &msg) = 0;

    controller_manager_msgs::ListControllersResponse listControllers()
    {
        ROS_INFO("Calling service list_controllers ...");
        if (srv_contr_list_.call(lc_))
        {
            ROS_INFO("Successfully called service list_controllers");
            controller_manager_msgs::ListControllersResponse controller_list;
            controller_list = lc_.response;
            cout << "\tLoaded controllers: " << endl;
            for (int i=0; i<controller_list.controller.size(); i++) {
                std::string controller_name = controller_list.controller.at(i).name;
                cout << controller_name << ", ";
            }
            cout << endl;
            return controller_list;
        }
        else
        {
            ROS_ERROR("Failed to call service list_controllers");
            exit(1);
        }
    }


    std::vector<std::string> getRunningControllers()
    {
        controller_manager_msgs::ListControllersResponse controller_list = listControllers();
        std::vector<std::string> running_controllers;

        for (int i=0; i<controller_list.controller.size(); i++) {
            std::string controller_name = controller_list.controller.at(i).name;
            if (controller_name.compare(this->controller_name_) == 0) {
                if (std::string(controller_list.controller.at(i).state).compare("running") == 0) {
                    running_controllers.push_back(controller_name);

                    ROS_DEBUG_STREAM("getRunningControllers(): Add " << controller_list.controller.at(i) << " to list running_controllers");
                }
            }
        }

        return running_controllers;
    }

    bool checkController()
    {
        std::vector<std::string> running_controllers = getRunningControllers();
        bool right_contr = false;

        ROS_INFO("Checking, if right controller loaded ...");

        for (std::vector<std::string>::const_iterator it = running_controllers.begin(); it != running_controllers.end(); ++it) {
            std::string controller_name = *it;

            if (controller_name.compare(this->controller_name_) == 0) {
                    cout << "\tThe required controller is already running: " << controller_name_ << endl;
                    right_contr = true;
            }
        }
        if (!right_contr) {
            cout << "\tThe required controller need to be started: " << controller_name_ << endl;
        }

        return right_contr;
    }

    /*
    bool checkController()
    {
        controller_manager_msgs::ListControllersResponse controller_list = listControllers();
        bool right_contr = false;

        ROS_INFO("Checking, if right controller loaded ...");
        for (int i=0; i<controller_list.controller.size(); i++) {
            std::string controller_name = controller_list.controller.at(i).name;

            if (controller_name.compare(this->controller_name_) == 0) {
                if (std::string(controller_list.controller.at(i).state).compare("running") == 0) {
                    cout << "\tThe required controller is already running: " << controller_name_ << endl;
                    right_contr = true;
                }
            }
        }
        if (!right_contr) {
            cout << "\tThe required controller need to be started: " << controller_name_ << endl;
        }

        return right_contr;
    }
    */

    sensor_msgs::JointState getJointState()
    {
        return js_;
    }

    bool isInitialized()
    {
        return initialized_;
    }

    std::string getControllerName()
    {
        return this->controller_name_;
    }

    bool switchController(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers, int strictness = 1)
    {
        //start_controllers.push_back("itr_joint_impedance_controller");
        //stop_controllers.push_back("joint_trajectory_controller");

        sc_.request.start_controllers = start_controllers;
        sc_.request.stop_controllers = stop_controllers;
        sc_.request.strictness = strictness;
        int ret_val = 0;

        ROS_INFO("Request to START these controllers: ");
        for(std::vector<std::string>::const_iterator it = start_controllers.begin(); it != start_controllers.end(); ++it) {
           cout << "\t" << (*it) << ", ";
        }
        cout << endl;

        ROS_INFO("Request to STOP these controllers: ");
        for(std::vector<std::string>::const_iterator it= stop_controllers.begin(); it != stop_controllers.end(); ++it) {
           cout << "\t" << (*it) << ", ";
        }
        cout << endl;

        ROS_INFO("Calling service switch_controller ...");
        if (srv_contr_switcher_.call(sc_))
        {
            ROS_INFO("Successfully called service switch_controller");
        }
        else
        {
            ROS_ERROR("Failed to call service switch_controller");
            ret_val = 1;
        }

        //let the KRC some time to switch control mode:
        ROS_INFO("Waiting for KRC ...");
        sleep(3);   // TODO find good time value for all controllers
        ROS_INFO("Switch Done!");

        return ret_val;
    }

    virtual ~testController()
    {    }

};

class cartController : public testController {
private:


public:

    cartController(ros::NodeHandle &nh, std::string controller_name) : testController(nh, controller_name)
    {
        initialized_ = false;

        ROS_INFO("Subscribing to /lwr/joint_states...");
        sub_js_ = nh_.subscribe ("/lwr/joint_states", 10, &cartController::callbackJointState, this);

        ROS_INFO_STREAM ("Subscribing to /lwr/" << controller_name_ << "/measured_cartesian_position...");
        sub_js_ = nh_.subscribe ("/lwr/" + controller_name_ + "/measured_cartesian_position", 10, &cartController::callbackCartPosition, this);

        pub_pose_ = nh_.advertise<lwr_controllers::PoseRPY>("/lwr/" + controller_name_ + "/position", 500);
        pub_force_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/force", 500);
        pub_gains_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/gains", 500);

        ROS_INFO("Creating service handle: switch_controller");
        srv_contr_switcher_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("lwr/controller_manager/switch_controller");
        ROS_INFO("Creating service handle: list_controllers");
        srv_contr_list_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("lwr/controller_manager/list_controllers");

        // if required controller not running, switch controller
        if (!checkController()) {
            std::vector<std::string> start_controllers;
            start_controllers.push_back(controller_name_);
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("joint_trajectory_controller");  // TODO which controllers need to be stopped?
            switchController(start_controllers, stop_controllers);
        }
    }

    void callbackCartPosition(const lwr_controllers::PoseRPYConstPtr &msg)
    {
        cart_pose_ = *msg;
        initialized_ = true;
    }

    void callbackJointState(const sensor_msgs::JointStateConstPtr &msg)
    {
        js_ = *msg;
        initialized_ = true;
    }

    lwr_controllers::PoseRPY getCartPosition()
    {
        return cart_pose_;
    }

    lwr_controllers::PoseRPY setCartPosition(double x, double y, double z, double roll, double pitch, double yaw)
    {
        lwr_controllers::PoseRPY pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.y = z;
        pose.orientation.roll = roll;
        pose.orientation.pitch = pitch;
        pose.orientation.yaw = yaw;
        return pose;
    }

    std_msgs::Float64MultiArray setCartGainsVector(float stiffness, float damping)
    {
        std_msgs::Float64MultiArray cart_vector;

        for (int i=0; i< NUM_CART_DIM; i++ ) {
            cart_vector.data.push_back(stiffness);
        }
        for (int i=NUM_CART_DIM; i< 2*NUM_CART_DIM; i++ ) {
            cart_vector.data.push_back(damping);
        }
        return cart_vector;
    }

    ~cartController() {}

};

class jointController : public testController {
private:


public:

    jointController(ros::NodeHandle &nh, std::string controller_name) : testController(nh, controller_name)
    {
        initialized_ = false;

        ROS_INFO("Subscribing to /lwr/joint_states...");
        sub_js_ = nh_.subscribe ("/lwr/joint_states", 10, &jointController::callbackJointState, this);

        pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/position", 500);
        pub_force_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/force", 500);
        pub_gains_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/gains", 500);

        ROS_INFO("Creating service handle: switch_controller");
        srv_contr_switcher_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("lwr/controller_manager/switch_controller");
        ROS_INFO("Creating service handle: list_controllers");
        srv_contr_list_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("lwr/controller_manager/list_controllers");

        // if required controller not running, switch controller
        if (!checkController()) {
            std::vector<std::string> start_controllers;
            start_controllers.push_back(controller_name_);
            std::vector<std::string> stop_controllers;
            stop_controllers.push_back("joint_trajectory_controller");  // TODO which controllers need to be stopped?
            switchController(start_controllers, stop_controllers);
        }
    }

    void callbackJointState(const sensor_msgs::JointStateConstPtr &msg)
    {
        js_ = *msg;
        initialized_ = true;
    }

    std_msgs::Float64MultiArray setJointVector(float val)
    {
        std_msgs::Float64MultiArray joint_vector;
        for (int i=0; i<NUM_LWR_JOINTS; i++ ) {
            joint_vector.data.push_back(val);
        }
        return joint_vector;
    }

    std_msgs::Float64MultiArray setJointGainsVector(float stiffness, float damping)
    {
        std_msgs::Float64MultiArray joint_vector;

        for (int i=0; i< NUM_LWR_JOINTS; i++ ) {
            joint_vector.data.push_back(stiffness);
        }
        for (int i=NUM_LWR_JOINTS; i< 2*NUM_LWR_JOINTS; i++ ) {
            joint_vector.data.push_back(damping);
        }
        return joint_vector;
    }

    ~jointController() {}

};
