#include "ros/ros.h"
#include "std_msgs/Float64.h"


int main(int argc, char** argv){
    
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle nh;
    //pub1 and 2

    ros::Publisher joint_1_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_pos_controller/command", 10);
    ros::Publisher joint_2_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_pos_controller/command", 10);

    ros::Rate rate(10);

    int start_time{0}, elapsed{0};
    while(! start_time){
        start_time = ros::Time::now().toSec();
    }

    std_msgs::Float64 joint_1_angle;
    std_msgs::Float64 joint_2_angle;

    while (ros::ok())
    {
        elapsed = ros::Time::now().toSec();
        joint_1_angle.data = M_PI_2 * 0.1 *sin(2*M_PI*elapsed);
        joint_1_angle.data = M_PI_2 * 0.1 *sin(2*M_PI*elapsed);
        
        joint_1_pub.publish(joint_1_angle);
        joint_2_pub.publish(joint_2_angle);

        rate.sleep();
    }
    
    return 0;
}