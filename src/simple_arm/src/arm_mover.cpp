
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include"simple_arm/GoToPosition.h"

ros::Publisher joint_1_pub, joint_2_pub;

std::vector<float> make_safe_joints(float req_joint_1, float req_joint_2){
    ros::NodeHandle nh2;
    float joint_1_min, joint_1_max, joint_2_min, joint_2_max;
    std::string node_name = ros::this_node::getName();

    // Read from the param server about the safe range min and max limits
    nh2.getParam(node_name + "/joint_1_min", joint_1_min);
    nh2.getParam(node_name + "/joint_1_max", joint_1_max);
    nh2.getParam(node_name + "/joint_2_min", joint_2_min);
    nh2.getParam(node_name + "/joint_2_max", joint_2_max);

    std::vector<float> safe_joints{};
    float safe_j_1,safe_j_2;
    // If joint 1 out of safe range, limit it
    if (req_joint_1 < joint_1_min || req_joint_1 > joint_1_max) {
        safe_joints.emplace_back(std::min(std::max(req_joint_1, joint_1_min), joint_1_max));
        ROS_WARN("joint 1 is out of the valid range: (%1.2f,%1.2f), limited to: %1.2f", joint_1_min, joint_1_max, safe_joints.front());
    }
    else{
        safe_joints.emplace_back(req_joint_1);
    }

    // If joint 2 out of safe range, limit it
    if (req_joint_2 < joint_2_min || req_joint_2 > joint_2_max) {
        safe_joints.emplace_back(std::min(std::max(req_joint_2, joint_2_min), joint_2_max));
        ROS_WARN("joint 2 is out of the valid range: (%1.2f,%1.2f), limited to: %1.2f", joint_2_min, joint_2_max, safe_joints.back());
    }
    else{
        safe_joints.emplace_back(req_joint_2);
    }
    return safe_joints;
}

bool safe_move_callback(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res){
    std::vector<float> safe_joints = make_safe_joints(req.joint_1, req.joint_2);

    std_msgs::Float64 safe_joint_1;
    std_msgs::Float64 safe_joint_2;
    safe_joint_1.data = safe_joints[0];
    safe_joint_2.data = safe_joints[1];
    joint_1_pub.publish(safe_joint_1);
    joint_2_pub.publish(safe_joint_2);

    ros::Duration(3).sleep();

    res.msg_feedback = "Joint angles set to joint_1 = " + std::to_string(safe_joints[0]) + ", and joint_2 = " + std::to_string(safe_joints[1]);
    return true;
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle nh;

    joint_1_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint_2_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
    
    ros::ServiceServer service = nh.advertiseService("/arm_mover/safe_move", safe_move_callback);

    ROS_INFO_STREAM("Node " << ros::this_node::getName() << "/safe_move service is" << " ready to receive joint commands ...");

    ros::spin();
}