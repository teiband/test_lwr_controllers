#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager/controller_loader.h>
#include <controller_interface/controller.h>

#include <tf/transform_listener.h>

// #include <lwr_controllers/PoseRPY.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#define NUM_LWR_JOINTS      7
#define NUM_CART_DIM        6

using namespace std;

class testController
{
protected:
	
    sensor_msgs::JointState js_;
    bool initialized_;
    // lwr_controllers::PoseRPY cart_pose_;
    geometry_msgs::PoseStamped cart_pose_;
    ros::Subscriber sub_js_;
    ros::Subscriber sub_pose_;
    controller_manager_msgs::SwitchController sc_;
    controller_manager_msgs::ListControllers lc_;
    ros::ServiceClient srv_contr_switcher_;
    ros::ServiceClient srv_contr_list_;
    std::string controller_name_;
    ros::NodeHandle nh_;
    tf::StampedTransform world_transform_;

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
    tf::TransformListener tf_listener_;

public:

    ros::Publisher pub_pose_world_;

    cartController(ros::NodeHandle &nh, std::string controller_name) : testController(nh, controller_name)
    {
        initialized_ = false;

        ROS_INFO("Subscribing to /lwr/joint_states...");
        sub_js_ = nh_.subscribe ("/lwr/joint_states", 10, &cartController::callbackJointState, this);

        ROS_INFO_STREAM ("Subscribing to /lwr/" << controller_name_ << "/measured_cartesian_position...");
        sub_pose_ = nh_.subscribe ("/lwr/" + controller_name_ + "/measured_cartesian_pose", 10, &cartController::callbackCartPosition, this);

        ROS_INFO_STREAM ("Looking up Transform...");

        bool init_transform = true;
        while (nh_.ok() && init_transform) {
            try {
                ros::Time ros_time = ros::Time(0);
                tf_listener_.lookupTransform("box", "lwr_7_link", ros_time, world_transform_);
                init_transform = false;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        ROS_INFO_STREAM ("done!");

        // TODO find topic name convention for ALL controllers
        pub_pose_ = nh_.advertise<geometry_msgs::Pose>("/lwr/" + controller_name_ + "/pose", 500);
        pub_pose_world_ = nh_.advertise<geometry_msgs::Pose>("/lwr/" + controller_name_ + "/pose_world", 500);
        pub_force_ = nh_.advertise<std_msgs::Float64MultiArray> ("/lwr/" + controller_name_ + "/addFT", 500);
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

    void callbackCartPosition(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        cart_pose_ = *msg;
        initialized_ = true;
    }

//    void callbackCartPosition(const lwr_controllers::PoseRPYConstPtr &msg)
//    {
//        cart_pose_ = *msg;
//        initialized_ = true;
//    }

    void callbackJointState(const sensor_msgs::JointStateConstPtr &msg)
    {
        js_ = *msg;
        initialized_ = true;
    }

    geometry_msgs::PoseStamped getCartPosition()
    {
        return cart_pose_;
    }

    geometry_msgs::PoseStamped getWorldPosition()
    {
        geometry_msgs::PoseStamped tempPose;
        while (nh_.ok()) {
            try {
                tf_listener_.lookupTransform("box", "lwr_7_link", ros::Time(0), world_transform_);

                tempPose.pose.position.x = world_transform_.getOrigin().x();
                tempPose.pose.position.y = world_transform_.getOrigin().y();
                tempPose.pose.position.z = world_transform_.getOrigin().z();
                tempPose.pose.orientation.x = world_transform_.getRotation().x();
                tempPose.pose.orientation.y = world_transform_.getRotation().y();
                tempPose.pose.orientation.z = world_transform_.getRotation().z();
                tempPose.pose.orientation.w = world_transform_.getRotation().w();

                return tempPose;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
            }
        }
    }

//    lwr_controllers::PoseRPY getCartPosition()
//    {
//        return cart_pose_;
//    }

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

    std_msgs::Float64MultiArray setCartGainsVector(float pos_stiffness, float orientation_stiffness, float damping)
    {
        std_msgs::Float64MultiArray cart_vector;
        cart_vector.data.resize(2*NUM_CART_DIM);

        for (int i=0; i< 3; i++ ) {
            cart_vector.data[i] = pos_stiffness;
            cart_vector.data[i+3] = pos_stiffness;
        }
        for (int i=0; i< 6; i++ ) {
            cart_vector.data[i+6] = damping;

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
