#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "joy_feedback");
    ros::NodeHandle nh;
    ros::Publisher joy_pub = nh.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback",10);

    ros::Rate loop_rate(50); 
    while(ros::ok())
    {
        sensor_msgs::JoyFeedbackArray feedback_array;
        sensor_msgs::JoyFeedback fb1;
        fb1.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
        fb1.id = 1;
        fb1.intensity = 80.8;

        feedback_array.array.push_back(fb1);
        joy_pub.publish(feedback_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
}