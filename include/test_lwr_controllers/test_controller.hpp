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

class TestController
{
protected:
	
    sensor_msgs::JointState js_;
    bool initialized_;
    geometry_msgs::PoseStamped cart_pose_;
    ros::Subscriber sub_js_;
    ros::Subscriber sub_pose_;
    controller_manager_msgs::SwitchController sc_;
    controller_manager_msgs::ListControllers lc_;
    ros::ServiceClient srv_contr_switcher_;
    ros::ServiceClient srv_contr_list_;
    std::string controller_name_;
    std::string position_controller_name_;
    std::string lwr_side_;
    std::string topic_joint_states_;
    ros::NodeHandle nh_;
    tf::StampedTransform base_link_transform_;
    tf::TransformListener tf_listener_;

    std::string lwr_ns_; // robot namespace
    std::string base_link_;
    std::string lwr_base_link_;
    std::string tip_link_;

public:

    ros::Publisher pub_pose_;
    ros::Publisher pub_pose_base_link_;
    ros::Publisher pub_force_;
    ros::Publisher pub_gains_;

    TestController(ros::NodeHandle &nh, std::string controller_name, std::string position_controller_name, std::string lwr_side) :
        nh_(nh),
        controller_name_(controller_name),
        position_controller_name_(position_controller_name),
        lwr_side_(lwr_side)
    {
        if (lwr_side_.compare("left") != 0 && lwr_side_.compare("right") != 0) {
            ROS_ERROR("Specifiy a side, which KUKA LWR arm you want to use with the keywords 'left' or 'right'");
            exit(EXIT_FAILURE);
        }
        ROS_INFO_STREAM("Using the " << lwr_side_ << " hand side KUKA LWR");

        lwr_ns_ = "/lwr_" + lwr_side_;

        topic_joint_states_ = lwr_ns_ + "/joint_states";

        base_link_ = "base_link";
        lwr_base_link_ = lwr_ns_ + "_base_link";
        tip_link_ = lwr_ns_ + "_7_link";

    }

    virtual void callbackJointState(const sensor_msgs::JointStateConstPtr &msg) = 0;

    controller_manager_msgs::ListControllersResponse listControllers()
    {
        ROS_INFO("Calling service list_controllers ...");
        if (srv_contr_list_.call(lc_))
        {
            //ROS_INFO("Successfully called service list_controllers");
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
            exit(EXIT_FAILURE);
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

    sensor_msgs::JointState getJointState()
    {
        return js_;
    }

    sensor_msgs::JointState::_position_type getJointPosition()
    {
        sensor_msgs::JointState temp_state;
        temp_state.position.resize(7);

        // copy the first 7 values, which are joint positions ...
        std::copy(js_.position.begin(), js_.position.begin() + NUM_LWR_JOINTS, temp_state.position.begin());

        return temp_state.position;
    }

    geometry_msgs::Pose createCartPoseRPY(double x, double y, double z, double roll, double pitch, double yaw)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.y = z;

        tf::Quaternion temp;
        temp.setRPY(roll, pitch, yaw);

        tf::quaternionTFToMsg(temp, pose.orientation);

        return pose;
    }

    geometry_msgs::PoseStamped getCartBaseLinkPosition()
    {
        geometry_msgs::PoseStamped tempPose;
        while (nh_.ok()) {
            try {
                tf_listener_.lookupTransform(base_link_, tip_link_, ros::Time(0), base_link_transform_);

                tempPose.pose.position.x = base_link_transform_.getOrigin().x();
                tempPose.pose.position.y = base_link_transform_.getOrigin().y();
                tempPose.pose.position.z = base_link_transform_.getOrigin().z();
                tempPose.pose.orientation.x = base_link_transform_.getRotation().x();
                tempPose.pose.orientation.y = base_link_transform_.getRotation().y();
                tempPose.pose.orientation.z = base_link_transform_.getRotation().z();
                tempPose.pose.orientation.w = base_link_transform_.getRotation().w();

                return tempPose;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
            }
        }
    }

    geometry_msgs::PoseStamped getCartRobotPosition()
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = tip_link_;
        pose.pose.position.x = js_.position[25];
        pose.pose.position.y = js_.position[26];
        pose.pose.position.z = js_.position[27];

        tf::Transform temp_tf(tf::Matrix3x3(js_.position[28], js_.position[29], js_.position[30],
                                            js_.position[31], js_.position[32], js_.position[33],
                                            js_.position[34], js_.position[35], js_.position[36]));

        tf::quaternionTFToMsg(temp_tf.getRotation(), pose.pose.orientation);

        return pose;
    }


    bool isInitialized()
    {
        return initialized_;
    }

    std::string getControllerName()
    {
        return this->controller_name_;
    }

    std::string getPositionControllerName()
    {
        return this->position_controller_name_;
    }

    bool switchController(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers, int strictness = 1)
    {
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
        sleep(5);   // TODO find good time value for all controllers
        ROS_INFO("Switch Done!");

        return ret_val;
    }

    bool switchController(std::string start_controller, std::string stop_controller, int strictness = 1)
    {
        std::vector<std::string> start_controllers;
        start_controllers.push_back(start_controller);
        std::vector<std::string> stop_controllers;
        stop_controllers.push_back(stop_controller);

        switchController(start_controllers, stop_controllers, strictness);

    }

    virtual ~TestController()
    {    }

};

class CartController : public TestController {
private:
    tf::TransformListener tf_listener_;

public:

    CartController(ros::NodeHandle &nh, std::string controller_name, std::string position_controller_name, std::string lwr_side) :
        TestController(nh, controller_name, position_controller_name, lwr_side)
    {
        initialized_ = false;

        ROS_INFO_STREAM("Subscribing to: " << topic_joint_states_);
        sub_js_ = nh_.subscribe (topic_joint_states_, 10, &CartController::callbackJointState, this);

        //ROS_INFO_STREAM ("Subscribing to /lwr/" << controller_name_ << "/measured_cartesian_position...");
        //sub_pose_ = nh_.subscribe ("/lwr/" + controller_name_ + "/measured_cartesian_pose", 10, &CartController::callbackCartPosition, this);

        ROS_INFO_STREAM ("Waiting for base_link Transform...");
        try {
            //ros::Time ros_time = ros::Time(0);
            tf_listener_.waitForTransform(base_link_, tip_link_, ros::Time(0), ros::Duration(5.0));
            tf_listener_.lookupTransform(base_link_, tip_link_, ros::Time(0), base_link_transform_);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        getCartBaseLinkPosition();
        ROS_INFO_STREAM ("done!");

        std::string topic_pose = lwr_ns_ + "/" + controller_name_ + "/pose";
        std::string topic_pose_base_link = lwr_ns_ + "/" + controller_name_ + "/pose_base_link";
        std::string topic_addFT = lwr_ns_ + "/" + controller_name_ + "/addFT";
        std::string topic_gains = lwr_ns_ + "/" + controller_name_ + "/gains";

        ROS_INFO_STREAM("Creating publishers to: " << endl <<
                        topic_pose << endl <<
                        topic_pose_base_link << endl <<
                        topic_addFT << endl <<
                        topic_gains);

        pub_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_pose, 500);
        pub_pose_base_link_ = nh_.advertise<geometry_msgs::Pose>(topic_pose_base_link, 500);
        pub_force_ = nh_.advertise<std_msgs::Float64MultiArray> (topic_addFT, 500);
        pub_gains_ = nh_.advertise<std_msgs::Float64MultiArray> (topic_gains, 500);

        ROS_INFO("Creating service handle: switch_controller");
        srv_contr_switcher_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(lwr_ns_ + "/controller_manager/switch_controller");
        ROS_INFO("Creating service handle: list_controllers");
        srv_contr_list_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(lwr_ns_ + "/controller_manager/list_controllers");

        // if required controller not running, switch controller
        if (!checkController()) {
            switchController(controller_name, position_controller_name);
        }
    }

    void callbackJointState(const sensor_msgs::JointStateConstPtr &msg)
    {
        js_ = *msg;
        initialized_ = true;
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

    ~CartController() {}

};

class JointController : public TestController {
private:

public:

    JointController(ros::NodeHandle &nh, std::string controller_name, std::string position_controller_name, std::string lwr_side) :
        TestController(nh, controller_name, position_controller_name, lwr_side)
    {
        initialized_ = false;

        ROS_INFO_STREAM("Subscribing to: " << lwr_ns_ << "/joint_states...");
        sub_js_ = nh_.subscribe (lwr_ns_ + "/joint_states", 10, &JointController::callbackJointState, this);

        std::string topic_position = lwr_ns_ + "/" + controller_name_ + "/position";
        std::string topic_force = lwr_ns_ + "/" + controller_name_ + "/force";
        std::string topic_gains = lwr_ns_ + "/" + controller_name_ + "/gains";

        ROS_INFO_STREAM("Creating publishers to: " << endl <<
                        topic_position << endl <<
                        topic_force << endl <<
                        topic_gains);

        pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray> (topic_position, 500);
        pub_force_ = nh_.advertise<std_msgs::Float64MultiArray> (topic_force, 500);
        pub_gains_ = nh_.advertise<std_msgs::Float64MultiArray> (topic_gains, 500);

        ROS_INFO("Creating service handle: switch_controller");
        srv_contr_switcher_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(lwr_ns_ + "/controller_manager/switch_controller");
        ROS_INFO("Creating service handle: list_controllers");
        srv_contr_list_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(lwr_ns_ + "/controller_manager/list_controllers");

        // if required controller not running, switch controller
        if (!checkController()) {
            switchController(controller_name, position_controller_name);
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
        joint_vector.data.resize(NUM_LWR_JOINTS);
        std::fill(joint_vector.data.begin(), joint_vector.data.end(), val);

        return joint_vector;
    }

    std_msgs::Float64MultiArray setJointGainsVector(float stiffness, float damping)
    {
        std_msgs::Float64MultiArray joint_vector;
        joint_vector.data.resize(2*NUM_LWR_JOINTS);
        std::fill(joint_vector.data.begin(), joint_vector.data.begin() + NUM_LWR_JOINTS-1, stiffness);
        std::fill(joint_vector.data.begin() + NUM_LWR_JOINTS, joint_vector.data.end(), stiffness);

        /*
        for (int i=0; i< NUM_LWR_JOINTS; i++ ) {
            joint_vector.data.push_back(stiffness);
        }
        for (int i=NUM_LWR_JOINTS; i< 2*NUM_LWR_JOINTS; i++ ) {
            joint_vector.data.push_back(damping);
        }
        */
        return joint_vector;
    }

    ~JointController() {}

};
