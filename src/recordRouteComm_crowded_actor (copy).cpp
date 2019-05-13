//
// Created by lucasyu on 18-11-29.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include "/home/ubuntu/catkin_ws/devel/include/darknet_ros_msgs/BoundingBoxes.h"

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

float person_standing[2] = {0.0, 0.0};
float person_standing_0[2] = {0.0, 0.0};
float actor_01_pos[2] = {0.0, 0.0};
float actor_02_pos[2] = {0.0, 0.0};
float actor_03_pos[2] = {0.0, 0.0};
float actor_04_pos[2] = {0.0, 0.0};
float actor_05_pos[2] = {0.0, 0.0};
float actor_06_pos[2] = {0.0, 0.0};
float actor_07_pos[2] = {0.0, 0.0};
float actor_08_pos[2] = {0.0, 0.0};

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
    control_angular, pos robot(x,y), vel robot(x,y), yaw robot, people_near,
     pos person_standing, pos person_standing_0, pos actor_01, pos actor_02, pos actor_03,
     pos actor_04, pos actor_05, pos actor_06, pos actor_07, pos actor_08
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

    write_xy(file, person_standing);

    write_xy(file, person_standing_0);

    write_xy(file, actor_01_pos);

    write_xy(file, actor_02_pos);

    write_xy(file, actor_03_pos);

    write_xy(file, actor_04_pos);

    write_xy(file, actor_05_pos);

    write_xy(file, actor_06_pos);

    write_xy(file, actor_07_pos);

    write_xy(file, actor_08_pos);

    write_endl(file);

}


void pos_diff_xy(const double& this_time, const double& last_time, const float pos[2], const float pos_last[2], float vel[2])
{
    double duration = this_time - last_time;
    // cout << duration << endl;
    vel[0] = (pos[0] - pos_last[0]) / duration;
    vel[1] = (pos[1] - pos_last[1]) / duration;
}


void callBackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->axes[7] == 1)
    {
        arr_command = 1;
    }
    else if (joy->axes[7] == -1)
    {
        arr_command = 2;
    }
    else if (joy->axes[6] == 1)
    {
        arr_command = 3;
    }
    else if (joy->axes[6] == -1)
    {
        arr_command = 4;
    }
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


void groundtruthCallback(const gazebo_msgs::ModelStates& msg)
{
    
    int num_model = msg.name.size();
        
    if (std::strcmp(msg.name[num_model-1].c_str(),"mobile_base")){
        cout << "Attention! Got the wrong object from model_states!! not the mobile base!!"<<endl 
             << "Object got from model_states: " << msg.name[num_model-1].c_str() << endl;
    }

    // update the data (pos, vel)
    robot_pos[0] = msg.pose[num_model-1].position.x;
    robot_pos[1] = msg.pose[num_model-1].position.y;
    
    actor_08_pos[0] = msg.pose[num_model-2].position.x;
    actor_08_pos[1] = msg.pose[num_model-2].position.y;
    actor_07_pos[0] = msg.pose[num_model-3].position.x;
    actor_07_pos[1] = msg.pose[num_model-3].position.y;
    actor_06_pos[0] = msg.pose[num_model-4].position.x;
    actor_06_pos[1] = msg.pose[num_model-4].position.y;
    actor_05_pos[0] = msg.pose[num_model-5].position.x;
    actor_05_pos[1] = msg.pose[num_model-5].position.y;
    actor_04_pos[0] = msg.pose[num_model-6].position.x;
    actor_04_pos[1] = msg.pose[num_model-6].position.y;
    actor_03_pos[0] = msg.pose[num_model-7].position.x;
    actor_03_pos[1] = msg.pose[num_model-7].position.y;
    actor_02_pos[0] = msg.pose[num_model-8].position.x;
    actor_02_pos[1] = msg.pose[num_model-8].position.y;
    actor_01_pos[0] = msg.pose[num_model-9].position.x;
    actor_01_pos[1] = msg.pose[num_model-9].position.y;
    person_standing_0[0] = msg.pose[num_model-5].position.x;
    person_standing_0[1] = msg.pose[num_model-5].position.y;
    person_standing[0] = msg.pose[num_model-6].position.x;
    person_standing[1] = msg.pose[num_model-6].position.y;

    float q0 = msg.pose[num_model-1].orientation.x;
    float q1 = msg.pose[num_model-1].orientation.y;
    float q2 = msg.pose[num_model-1].orientation.z;
    float q3 = msg.pose[num_model-1].orientation.w;

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
}


// TODO
void callbackCommand(const std_msgs::Float64MultiArray& cmd_msg)
{
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
    ros::Subscriber Joy_sub = nh.subscribe("/joy", 2, callBackJoy);
    ros::Subscriber yaw_sub = nh.subscribe("/gazebo/model_states",1,groundtruthCallback);
    ros::Subscriber direction_sub = nh.subscribe("/radar/direction", 1, callbackCommand);
    ros::Subscriber status_sub = nh.subscribe("/robot/status", 1, callbackRobotStatus);
    ros::Subscriber output_sub = nh.subscribe("/mobile_base/commands/velocity", 1, callbackOutput);
    ros::Subscriber objects_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, callbackObjects);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    // ros::Subscriber Cmd_sub = nh.subscribe("/mobile_base/commands/velocity", 2, callBackCmdMobileBase);

    ros::spin();

    outFile_route.close();
    // outFile_control.close();
    // outFile_command.close();

    return 0;
}

