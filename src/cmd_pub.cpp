#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <car_ros/CMD.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#define MAXSPEED 12
#define MAXYAW 50
//using namespace std; //声明命名空间
car_ros::CMD cmd;

void joy_sub_callback(const sensor_msgs::Joy::ConstPtr &joy)
{
     cmd.state = 0x01;
     cmd.vx = joy->axes[1] * MAXSPEED;
     cmd.yaw = -joy->axes[0] * MAXYAW;
}

int main(int argc, char** argv)
{
    //初始化，节点名为serial_publisher
    ros::init(argc, argv,"cmd_publisher");
    //创建句柄seuNB，用于管理资源
    ros::NodeHandle nh;

    //用Publisher类，实例化一个发布者对象
    ros::Publisher serial_pub = nh.advertise<car_ros::CMD>("serial_cmd",10);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 100, joy_sub_callback);

    ros::Rate loop_rate(50); //指定循环频率2
    while(ros::ok())
    {
        serial_pub.publish(cmd);  //将消息发布出去
        ROS_INFO("Publish the cmd message:"); //表明正在开始读取串口数据
        ROS_INFO("STATE: %d, VX: %f, V_YAW: %f\n", cmd.state,cmd.vx, cmd.yaw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
