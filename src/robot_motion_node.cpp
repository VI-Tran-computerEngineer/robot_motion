//#include "robot_motion_node.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>
#include <string.h>

#define MV_SPEED 2
#define ANGULAR_SPEED 40
#define PI 3.1416

using namespace std;

ros::Publisher velocity_publisher;
geometry_msgs::Twist vel_msg;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);
double degrees2radians(double angle_in_degrees);
bool checkData(string &msg, string &type);
bool msgDecoder(string msg, double &distance, bool &forward, double &angle, bool &clockwise);
bool isNumeric(string str);
void chatterCallback(const std_msgs::String::ConstPtr &msg);

int main(int argc, char **argv)
{
    // Initiate new ROS node named "talker"
    ros::init(argc, argv, "robot_motion");
    ros::NodeHandle n;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(10);

    //	/turtle1/cmd_vel is the Topic name
    //	/geometry_msgs::Twist is the msg type
    ROS_INFO("\n\n\n ******** START *********\n");

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    return 0;
}

void move(double speed, double distance, bool isForward)
{
    //set a random linear velocity in the x-axis
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    if (isForward)
        vel_msg.linear.x = abs(speed);
    else
        vel_msg.linear.x = -abs(speed);

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;
    ros::Rate loop_rate(100);
    do
    {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
        //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
    } while (current_distance < distance);
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
    //set a random linear velocity in the x-axis
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    if (clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;
    ros::Rate loop_rate(1000);
    do
    {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
        //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
    } while (current_angle < relative_angle);
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * PI / 180.0;
}

bool checkData(string &msg, string &type)
{
    std::stringstream spliter;
    spliter << msg;
    string temp;
    spliter >> temp;
    if (temp == "MOVE" || temp == "STOP")
        type = temp;
    else
        return false;
    msg = "";
    while (!spliter.eof())
    {
        spliter >> temp;
        if (msg.empty())
            msg = temp;
        else
            msg = msg + " " + temp;
    }
    return true;
}

bool isNumeric(string str)
{
    bool flag = 0;
    for (int i = 0; i < str.length(); i++)
    {
        if (str[i] == '.' && flag == 0)
        {
            flag = 1;
            continue;
        }
        if (isdigit(str[i]) == false)
            return false;
    }
    return true;
}

bool msgDecoder(string msg, double &distance, bool &forward, double &angle, bool &clockwise)
{
    std::stringstream spliter;
    spliter << msg;
    string temp;
    spliter >> temp;
    if (!isNumeric(temp))
        return false;
    distance = stod(temp);
    temp = "";
    spliter >> temp;
    if (temp == "true")
        forward = true;
    else if (temp == "false")
        forward = false;
    else
        return false;
    spliter >> temp;
    if (!isNumeric(temp))
        return false;
    angle = stod(temp);
    temp = "";
    spliter >> temp;
    if (temp == "true")
        clockwise = true;
    else if (temp == "false")
        clockwise = false;
    else
        return false;
    return true;
}

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    std::string msg_subscribed = msg->data.c_str();
    double distance, angle;
    bool isForward, clockwise;
    string type = "";
    if (!checkData(msg_subscribed, type))
        cout << "Message received fail!\n";
    else
    {
        if (type == "STOP")
        {
            distance = 0;
            isForward = true;
            angle = 0;
            clockwise = true;
        }
        else if (type == "MOVE")
            if (!msgDecoder(msg_subscribed, distance, isForward, angle, clockwise))
                cout << "Message type received fail!\n";
    }

    if (distance != 0)
        move(MV_SPEED, distance, isForward);
    if (angle != 0)
        rotate(degrees2radians(ANGULAR_SPEED), degrees2radians(angle), clockwise);
}
