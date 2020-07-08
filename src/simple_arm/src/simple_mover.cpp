#include <ros/ros.h>
#include <std_msgs/Float64.h>


int main (int argc, char ** argv){

    ros::init(argc, argv, "simple_mover2");
    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/simple_arm/jnt_1_pos_controller/command", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/simple_arm/jnt_2_pos_controller/command", 10);

    ros::Rate rate(10);

    int start_time{0}, elapsed_time{0};

    while (!start_time)
    {
        start_time = ros::Time::now().toSec();
    }

    std_msgs::Float64 jnt_1_val;
    std_msgs::Float64 jnt_2_val;

    ROS_INFO("Started jnts publisher");

    while(ros::ok()){
        elapsed_time = ros::Time::now().toSec() - start_time;

        jnt_1_val.data = M_PI_2 * sin(2*M_PI * elapsed_time);
        jnt_2_val.data = M_PI_2 * sin(2*M_PI * elapsed_time);
        
        pub1.publish(jnt_1_val);
        pub2.publish(jnt_2_val);

        rate.sleep();
    }

    return 0;
}