//
// Created by lucasyu on 18-11-29.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include "/home/gg/intel_nav_ws/devel/include/darknet_ros_msgs/BoundingBoxes.h"

#include <termios.h>  
#include <signal.h>  
#include <math.h>  	
#include <stdio.h>  
#include <stdlib.h>  
 
#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include <numeric>

using namespace std;

ofstream outFile_route;
// ofstream outFile_command;
// ofstream outFile_control;

float robot_pos[2] = {0.0, 0.0}; // x; y
// float robot_pos_last[2] = {0.0, 0.0};
float robot_vel[2] = {0.0, 0.0};

float linear_v = 0.f;
float angular_v = 0.f;

float robot_yaw = 0.0;
float status_last = 0.0;
float status_this = 0.0;

double duration_total_auto = 0.0;
double last_time = 0.0;
bool isFirstFlag = true;
float isAutoControl = 0.f; //0: false 1: true
float people_near = 0.f;

float arr_command = 0; // 0:idle 1:fwd 2:bckwd 3:lftwd 4:rightwd
// float arr_control[2] = {0.0, 0.0}; // linear vel; angular vel
float min_person_dist = 0.f;

void write_xy(ofstream& file, const float* arr)
{
    char c_data0[80];
    char c_data1[80];
    sprintf(c_data0, "%f", *arr);
    sprintf(c_data1, "%f", *(arr+1));
    file << c_data0 << "," << c_data1 << ",";
}


void write_float(ofstream& file, const float& data)
{
    char c_data1[80];
    sprintf(c_data1, "%f", data);
    file << c_data1 <<",";
}


void write_endl(ofstream& file)
{
    file << endl;
}


void writeCsvFromFloatArrs(ofstream& file)
{
    /* data order: 
    timestamp, flag Auto/manual, total duration Auto, 4commands, control_vel, 
    control_angular, pos robot(x,y), vel robot(x,y), yaw robot, people_near
    */
    double time_stamp = ros::Time::now().toSec();

    write_float(file, time_stamp);

    write_float(file, isAutoControl);

    write_float(file, duration_total_auto);

    write_float(file, arr_command);

    write_float(file, linear_v);

    write_float(file, angular_v);

    write_xy(file, robot_pos);

    write_xy(file, robot_vel);

    write_float(file, robot_yaw);
    
    write_float(file, people_near);
    people_near = 0.0;

    write_float(file, min_person_dist);

    write_endl(file);

}


void pos_diff_xy(const double& this_time, const double& last_time, const float pos[2], const float pos_last[2], float vel[2])
{
    double duration = this_time - last_time;
    // cout << duration << endl;
    vel[0] = (pos[0] - pos_last[0]) / duration;
    vel[1] = (pos[1] - pos_last[1]) / duration;
}

// void callBackCmdMobileBase(const geometry_msgs::Twist::ConstPtr& data)
// {
//     arr_control[0] = data->linear.x;
//     arr_control[1] = data->angular.z;
// }


void callbackRobotStatus(const std_msgs::Float64& msg)
{
    status_this = msg.data;
}


void odomCallback(const nav_msgs::Odometry& msg)
{
    // update the data (pos, vel)
    robot_pos[0] = msg.pose.pose.position.x;
    robot_pos[1] = msg.pose.pose.position.y;
    
    cout << "robot position: " << robot_pos[0] << robot_pos[1] << endl;
    float q0 = msg.pose.pose.orientation.x;
    float q1 = msg.pose.pose.orientation.y;
    float q2 = msg.pose.pose.orientation.z;
    float q3 = msg.pose.pose.orientation.w;

    /* Pitch roll may be needed for MAVs */
    robot_yaw = atan2(2*q3*q2 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);  // Yaw

}


void timerCallback(const ros::TimerEvent& event)
{
    static float robot_pos_last[2];
    if (isFirstFlag)
    {
        last_time = ros::Time::now().toSec();
        robot_pos_last[0] = robot_pos[0];
        robot_pos_last[1] = robot_pos[1];
        isFirstFlag = false;
        return;
    }
    double this_time = ros::Time::now().toSec(); 

    if (status_this != status_last) // autonomous mode is running
    {
        isAutoControl = 1.f;

         // add this duration to total autonomous duration
        double this_duration_sec = this_time - last_time;
        duration_total_auto += this_duration_sec;

    }
    else // manual control
    {
        isAutoControl = 0.f;
    }

    // calculate velocity
    pos_diff_xy(this_time, last_time, robot_pos, robot_pos_last, robot_vel);

    robot_pos_last[0] = robot_pos[0];
    robot_pos_last[1] = robot_pos[1];
    // write data

    writeCsvFromFloatArrs(outFile_route);

    status_last = status_this;
    last_time = this_time;
    people_near = 0.f;
}


void callbackOutput(const geometry_msgs::Twist& output)
{
    linear_v = output.linear.x;
    angular_v = output.angular.z;
    cout << "output: " << linear_v << ", " << angular_v << endl;
}


// TODO
void callbackCommand(const std_msgs::Float64MultiArray& cmd_msg)
{
    cout << "command: " << cmd_msg.data[0] << ", " << cmd_msg.data[1] << ", "
         << cmd_msg.data[2] << ", "<< cmd_msg.data[3] << ", " << endl;
    if(cmd_msg.data[0] == 1.0){
        arr_command = 1;
    }
    else if(cmd_msg.data[1] == 1.0){
        arr_command = 2;
    }
    else if(cmd_msg.data[2] == 1.0){
        arr_command = 3;
    }
    else if(cmd_msg.data[3] == 1.0)
    {
        arr_command = 4;
    }
    else{
        arr_command = 0;
    }

}


void callbackObjects(const darknet_ros_msgs::BoundingBoxes& objects)
{
    for (int m = 0; m < objects.bounding_boxes.size(); m++) {
        int label;
        /*
        0: free space
        1: unknown
        2: possible way
        3: obstacle
        4: none
        5: furniture
        6: other dynamic objects
        7: person
        */
        if (objects.bounding_boxes[m].Class == "person")
        {
            label = 7;
            people_near = 1.f;
        }
    }
}

void callbackPersonDist(const std_msgs::Float64& msg)
{
    min_person_dist = msg.data;
}

int main(int argc, char** argv)
{
    time_t t = time(0);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y_%m_%d_%X",localtime(&t) );

    char c_route_data[100];
    sprintf(c_route_data,"routeCommand_%s.csv",tmp);
    cout<<"----- file routeCommand_ data: "<< c_route_data<<endl;

    // char c_command_data[100];
    // sprintf(c_command_data,"command_data_%s.csv",tmp);
    // cout<<"----- file command data: "<< c_command_data<<endl;

    // char c_control_data[100];
    // sprintf(c_control_data,"control_data_%s.csv",tmp);
    // cout<<"----- file control data: "<< c_control_data<<endl;
    
    outFile_route.open(c_route_data, ios::out);
    // outFile_command.open(c_command_data, ios::out);
    // outFile_control.open(c_control_data, ios::out);


    ros::init(argc, argv, "recordRoute");
    ros::NodeHandle nh;
    ros::Subscriber yaw_sub = nh.subscribe("/EAI_e1_car/odom",1,odomCallback);
    ros::Subscriber direction_sub = nh.subscribe("/radar/direction", 1, callbackCommand);
    ros::Subscriber status_sub = nh.subscribe("/robot/status", 1, callbackRobotStatus);
    ros::Subscriber output_sub = nh.subscribe("smoother_cmd_vel", 1, callbackOutput);
    ros::Subscriber objects_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, callbackObjects);
    ros::Subscriber person_dist_sub = nh.subscribe("/person_dist", 1, callbackPersonDist);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    // ros::Subscriber Cmd_sub = nh.subscribe("/mobile_base/commands/velocity", 2, callBackCmdMobileBase);

    ros::spin();

    outFile_route.close();
    // outFile_control.close();
    // outFile_command.close();

    return 0;
}

