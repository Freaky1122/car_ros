#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <car_ros/CMD.h>


//using namespace std; //声明命名空间


int main(int argc, char** argv)
{
    //初始化，节点名为serial_publisher
    ros::init(argc, argv,"cmd_publisher");
    //创建句柄seuNB，用于管理资源
    ros::NodeHandle nh;

    //用Publisher类，实例化一个发布者对象
    ros::Publisher serial_pub = nh.advertise<car_ros::CMD>("serial_cmd",10);


    car_ros::CMD cmd;
    ros::Rate loop_rate(20); //指定循环频率2

    while(ros::ok())
    {
        ROS_INFO_STREAM("Publish the cmd message:"); //表明正在开始读取串口数据
        
        cmd.vx = 280;
        cmd.yaw = 59;
        serial_pub.publish(cmd);  //将消息发布出去
        ROS_INFO("VX: %d, V_YAW: %d\n", cmd.vx, cmd.yaw);

        loop_rate.sleep();
    }

    return 0;
}
