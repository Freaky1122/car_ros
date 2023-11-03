#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <car_ros/CMD.h>
#include <car_ros/IMU.h>


using namespace std; //声明命名空间

#define FRAME_SIZE 12
#define RX_BUFFER_SIZE 1024

typedef union
{
    float data;
    uint8_t data8[4];
}data_u;

typedef struct 
{
    uint8_t head;
    uint8_t udata[2];
    data_u fdata[4];
    uint8_t tail;
}MyFrame_t;

//实例化一个serial类
serial::Serial mySerial;

uint8_t rx_buffer[RX_BUFFER_SIZE];
MyFrame_t frame;

//函数功能：将数据经由串口发送出去
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  要通过串口发送出去的字符串
int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}

//函数功能：将从串口接收到的数据保存到数组中
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  从串口读取的字符串
int serial_read(serial::Serial &ser, std::string &serial_msg)
{
    serial_msg = ser.read( ser.available() );
    return 0;
}

void sub_callback(const car_ros::CMD::ConstPtr &cmd)
{
    ROS_INFO("CMD INFO : %d, %f,  %f", cmd->state, cmd->vx, cmd->yaw);
    ROS_INFO("Sending cmd message to the car :");

    data_u _temp;
    uint8_t data_to_send[FRAME_SIZE];
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0x9A;
    data_to_send[_cnt++] = 0xB4;
    data_to_send[_cnt++] = cmd->state;

    _temp.data = cmd->vx;
    data_to_send[_cnt++] = _temp.data8[0];
    data_to_send[_cnt++] = _temp.data8[1];
    data_to_send[_cnt++] = _temp.data8[2];
    data_to_send[_cnt++] = _temp.data8[3];

    _temp.data = cmd->yaw;
    data_to_send[_cnt++] = _temp.data8[0];
    data_to_send[_cnt++] = _temp.data8[1];
    data_to_send[_cnt++] = _temp.data8[2];
    data_to_send[_cnt++] = _temp.data8[3];

    data_to_send[_cnt++] = 0x4F;
    mySerial.write(data_to_send,  _cnt);
}

int main(int argc, char** argv)
{
    //初始化，节点名为serial_publisher
    ros::init(argc, argv,"serial2car");
    //创建句柄seuNB，用于管理资源
    ros::NodeHandle nh;

    //用Publisher类，实例化一个发布者对象yao，发布一个名为"Serial_Topic"的话题，话题的消息类型为std_msgs::String，消息发布队列长度为10(注意话题名中间不能有空格)
    ros::Subscriber serial_sub = nh.subscribe<car_ros::CMD>("serial_cmd", 10, sub_callback);

    ros::Publisher imu_pub = nh.advertise<car_ros::IMU>("imu",10);
    //初始化串口相关设置
    mySerial.setPort("/dev/ttyUSB0");         //设置打开的串口名称
    mySerial.setBaudrate(115200);           //设置串口的波特率
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);  //创建timeout
    mySerial.setTimeout(timeout);                //设置串口的timeout

    //打开串口
    try
    {
        mySerial.open();         //打开串口
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Unable to open port.");        //打开串口失败，打印日志信息，然后结束程序
        return -1;
    }

    //判断串口是否成功打开
    if(mySerial.isOpen())
    { 
        ROS_INFO("Serial Port is opened.\n");    //成功打开串口，打印日志信息
    }
    else
    {
        return -1;  //打开串口失败，打印日志信息，然后结束程序
    }
    uint8_t my[sizeof(MyFrame_t)];

    ros::Rate loop_rate(50);   //指定循环频率50
    while(ros::ok())
    {
        car_ros::IMU imu;
        MyFrame_t m;

        if(mySerial.available())
        {
            size_t n = mySerial.read(rx_buffer, mySerial.available());
            int state = 0;
            uint8_t rx_data[18] = {0};
            for(int i = 0; i < n && i < RX_BUFFER_SIZE; i++)
            {
                if(state == 0 && rx_buffer[i] == 0x4B)
                    state++;
                else if(state == 1 && rx_buffer[i] == 0x7F )
                    state++;
                else if(state >= 2 &&  state < 20)
                {
                    rx_data[state - 2] = rx_buffer[i];
                    state++;
                    if(state == 20)
                    {
                        m.udata[0] = rx_data[0];
                        m.udata[1] = rx_data[1];
                        for(int k = 0; k < 4; k++)
                        {
                            m.fdata[0].data8[k] = rx_data[2 + k];
                            m.fdata[1].data8[k] = rx_data[6 + k];
                            m.fdata[2].data8[k] = rx_data[10 + k];
                            m.fdata[3].data8[k] = rx_data[14 + k];
                        }
                        ROS_INFO( "Recived Messages:%d , %d", m.udata[0], m.udata[1]);
                        ROS_INFO( "Recived Messages:%.2f , %.2f, %.2f, %.2f", m.fdata[0].data, m.fdata[1].data, m.fdata[2].data, m.fdata[3].data);
                        state =0;
                    }
                }
                else state = 0;
            }
        }
        imu.roll = m.fdata[0].data;
        imu.pitch = m.fdata[1].data;
        imu.yaw = m.fdata[2].data;
        imu_pub.publish(imu);

        ros::spinOnce();
        loop_rate.sleep();
    }
  
    //关闭串口
	mySerial.close();

    return 0;
}

