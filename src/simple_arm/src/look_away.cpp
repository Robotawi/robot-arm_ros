#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <simple_arm/GoToPosition.h>

// glbal variables to oversee the situation
bool arm_stopped{false};
bool uniform_image{false};
ros::ServiceClient client_safemove;
std::vector<double> last_joints{0, 0};
std::vector<double> current_joints{0, 0};
double tolerance{0.005};

void move_robot(){
    //call the safe_move service
    simple_arm::GoToPosition move_service;
    move_service.request.joint_1 = 1.57;
    move_service.request.joint_2 = 1.57;

    if (!client_safemove.call(move_service)){
        ROS_INFO_STREAM("Failed to call the "<< ros::this_node::getName() <<"/safe_move service");
    }
}
void camera_callback(const sensor_msgs::Image& img){
    //check if the image is the same color, or the arm is looking to the sky in gazebo
    for (auto& pixel : img.data){
        if (pixel - img.data[0] == 0){
            uniform_image = true;
            break; // no more check is needed
        }
    }
    ROS_INFO_STREAM("Uniform_image is " << uniform_image << ", and arm stopped is " << arm_stopped);
    if (uniform_image && arm_stopped){
        move_robot();
        ROS_INFO_STREAM("Robot move is called");
    }
}

void jnt_states_callback(const sensor_msgs::JointState& jnt_states){
    // check if current joint = previous joints to know if the arm is moving or not
    current_joints = jnt_states.position;
    if (abs(current_joints[0] - last_joints[0]) < tolerance && abs(current_joints[1] - last_joints[1]) < tolerance){
        arm_stopped = true;
    }
    else{
        arm_stopped = false;
        last_joints = current_joints;
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "look_away");
    ros::NodeHandle nh;

    ros::Subscriber sub_cam = nh.subscribe("rgb_camera/image_raw", 10, camera_callback);
    ros::Subscriber sub_jnts = nh.subscribe("/simple_arm/joint_states", 10, jnt_states_callback);

    client_safemove = nh.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    ros::spin();
    return 0;
}