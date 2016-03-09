#include <ros/ros.h>
#include <test_lwr_controllers/test_controller.h>


testController::testController(ros::NodeHandle &nh)
{
    js_.position.resize(7);
    initialized_ = false;

    sub_js_ = nh.subscribe ("/lwr/joint_states", 10, &testController::callbackJointState, this);
    ROS_INFO("subscribed to /lwr/joint_states");

    sub_js_ = nh.subscribe ("/lwr/cartesian_impedance/measured_cartesian_position", 10, &testController::callbackCartPosition, this);
    ROS_INFO("subscribed to /lwr/joint_states");

    pub_pos_ = nh.advertise<std_msgs::Float64MultiArray> ("/lwr/itr_joint_impedance_controller/position", 500);
    pub_torque_ = nh.advertise<std_msgs::Float64MultiArray> ("/lwr/itr_joint_impedance_controller/torque", 500);
    pub_gains_ = nh.advertise<std_msgs::Float64MultiArray> ("/lwr/itr_joint_impedance_controller/gains", 500);

    srv_contr_switcher_ = nh.serviceClient<controller_manager_msgs::SwitchController>("lwr/controller_manager/switch_controller");
}

void testController::callbackCartPosition(const lwr_controllers::PoseRPYConstPtr &msg)
{
    cart_pose_ = *msg;
    initialized_ = true;
}

lwr_controllers::PoseRPY testController::getCartPosition()
{
    return cart_pose_;
}

std_msgs::Float64MultiArray testController::setJointVector(float val)
{
    std_msgs::Float64MultiArray joint_vector;
    for (int i=0; i<NUM_LWR_JOINTS; i++ ) {
        joint_vector.data.push_back(val);
    }
    return joint_vector;
}

std_msgs::Float64MultiArray testController::setGainsVector(float stiffness, float damping)
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


sensor_msgs::JointState testController::getJointState()
{
    return js_;
}

bool testController::isInitialized()
{
    return initialized_;
}


void testController::callbackJointState(const sensor_msgs::JointStateConstPtr &msg)
{
    js_ = *msg;
    //        std::cout << "read joint position from joint state: " << std::endl;
    //        for (int i=0; i<7; i++)
    //            std::cout << js.position[i];

    //        std::cout << std::endl;
    initialized_ = true;
}

bool testController::switchController(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers, int strictness /* = 1 */)
{
    //start_controllers.push_back("itr_joint_impedance_controller");
    //stop_controllers.push_back("joint_trajectory_controller");

    sc_.request.start_controllers = start_controllers;
    sc_.request.stop_controllers = stop_controllers;
    sc_.request.strictness = strictness;

    ROS_INFO("Calling service switch_controller...");

    if (srv_contr_switcher_.call(sc_))
    {
        ROS_INFO("Successfully called service switch_controller");
    }
    else
    {
        ROS_ERROR("Failed to call service switch_controller");
        return 1;
    }

    //let the KRC some time to switch control mode:
    sleep(5);
}

testController::~testController()
{
    // TODO switch back control mode to position control here?
    std::vector<std::string> start_controllers;
    start_controllers.push_back("itr_joint_impedance_controller");
    std::vector<std::string> stop_controllers;
    stop_controllers.push_back("joint_trajectory_controller");
    switchController(start_controllers, stop_controllers);

}
