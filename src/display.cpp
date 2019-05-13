#include <ros/ros.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "/home/gg/intel_nav_ws/devel/include/darknet_ros_msgs/BoundingBoxes.h"
#include <string>
#include <vector>
#include <math.h>

#define MAX_DEPTH 4500  // mm
#define IMGWIDTH 640
#define IMGHEIGHT 480

using namespace std;

int mode = -1; //rgb 0, depth 1, depth_semantic 2
int direction = -1; // Forward 0, backward 1, leftward 2, rightward 3

double linear_v = 0.0;
double angular_v = 0.0;

const double linear_v_abs_max = 1.0;
const double angular_v_abs_max = 1.0;

typedef struct semantic_box{
    string name;
    cv::Point p1;  // left, top
    cv::Point p2;  // right, bottom
    cv::Scalar color;
}SemanticBox;

std::vector<SemanticBox> boxes;
cv::Mat depth_img_global;

ros::Publisher person_dist_pub;


void add_infos(cv::Mat& img)
{
    int image_width = img.cols;
    int image_height = img.rows;

    /// Add semantic objects through a mask image
    if(mode == 2)  // depth + semantic
    {
        unsigned long int objects_num = boxes.size();

        if(objects_num > 0)
        {
            cv::Mat overlay;
            img.copyTo(overlay);

            for(int i=0; i<objects_num; i++)
            {
                cv::rectangle(overlay, boxes[i].p1, boxes[i].p2, boxes[i].color, -1, 1 ,0);
                cv::putText(img, boxes[i].name, cv::Point(boxes[i].p1.x, boxes[i].p1.y+10), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 0), 1, CV_AA);
            }

            float alpha = 0.5; // transparency
            cv::addWeighted(overlay, alpha, img, 1-alpha, 0, img);
        }

    }

    /// Add panels
    // add output velocity
    int linear_l_x = 10;
    int linear_l_y = 40;
    int linear_r_x = 20;
    int linear_r_y = image_height - 40;

    int angular_l_x = 40;
    int angular_l_y = image_height - 20;
    int angular_r_x = image_width - 40;
    int angular_r_y = image_height - 10;

    int linear_range = linear_r_y - linear_l_y;
    int angular_range = angular_r_x - angular_l_x;

    int linear_v_bar_center_x = (linear_l_x + linear_r_x) / 2;
    auto linear_v_bar_center_y = (int)(-linear_v / linear_v_abs_max * linear_range / 2.0 + (linear_l_y + linear_r_y) / 2.0);
    auto angular_v_bar_center_x = (int)(-angular_v / angular_v_abs_max * angular_range / 2.0 + (angular_l_x + angular_r_x) / 2.0);
    int angular_v_bar_center_y = (angular_l_y + angular_r_y) / 2;

    int bar_size_half = 8;

    cv::rectangle(img, cv::Point(linear_l_x, linear_l_y), cv::Point(linear_r_x, linear_r_y), cv::Scalar(200, 20, 0), 2, 1 ,0);
    cv::rectangle(img, cv::Point(angular_l_x, angular_l_y), cv::Point(angular_r_x, angular_r_y), cv::Scalar(200, 20, 0), 2, 1 ,0);

    cv::rectangle(img, cv::Point(linear_v_bar_center_x - bar_size_half, linear_v_bar_center_y - bar_size_half),
                  cv::Point(linear_v_bar_center_x + bar_size_half, linear_v_bar_center_y + bar_size_half), cv::Scalar(0, 0, 255), -1, 1 ,0);
    cv::rectangle(img, cv::Point(angular_v_bar_center_x - bar_size_half, angular_v_bar_center_y - bar_size_half),
                  cv::Point(angular_v_bar_center_x + bar_size_half, angular_v_bar_center_y + bar_size_half), cv::Scalar(0, 255, 0), -1, 1 ,0);

    cv::Point p_text1 = cv::Point(60, image_height - 80);
    cv::Point p_text2 = cv::Point(60, image_height - 65);
    char v_lnr[256];
    char v_ang[256];
    sprintf(v_lnr, "Linear Speed: %lf", linear_v);
    sprintf(v_ang, "Angular Speed: %lf", angular_v);
    string text1 = v_lnr;
    string text2 = v_ang;

    cv::putText(img, text1, p_text1, cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 0), 1, CV_AA);
    cv::putText(img, text2, p_text2, cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 0), 1, CV_AA);

    // add commands panel
    if(direction < 0) return;

    int cube_4x4_left = image_width - 80;
    int cube_4x4_top = 20;
    int cube_4x4_cell_size = 20;
    int cube_p[4][4][2]; // row, col, (x,y)

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            cube_p[i][j][0] = cube_4x4_left + cube_4x4_cell_size * j;  //x
            cube_p[i][j][1] = cube_4x4_top + cube_4x4_cell_size * i;  //y
        }
    }

    cv::Point cell_forward_1 = cv::Point(cube_p[0][1][0], cube_p[0][1][1]);
    cv::Point cell_forward_2 = cv::Point(cube_p[1][2][0], cube_p[1][2][1]);
    cv::Point cell_backward_1 = cv::Point(cube_p[2][1][0], cube_p[2][1][1]);
    cv::Point cell_backward_2 = cv::Point(cube_p[3][2][0], cube_p[3][2][1]);

    cv::Point cell_leftward_1 = cv::Point(cube_p[1][0][0], cube_p[1][0][1]);
    cv::Point cell_leftward_2 = cv::Point(cube_p[2][1][0], cube_p[2][1][1]);
    cv::Point cell_rightward_1 = cv::Point(cube_p[1][2][0], cube_p[1][2][1]);
    cv::Point cell_rightward_2 = cv::Point(cube_p[2][3][0], cube_p[2][3][1]);

    cv::Scalar color_not_pressed = cv::Scalar(35, 35, 139);
    cv::rectangle(img, cell_forward_1, cell_forward_2, color_not_pressed, -1, 1 ,0);
    cv::rectangle(img, cell_backward_1, cell_backward_2, color_not_pressed, -1, 1 ,0);
    cv::rectangle(img, cell_leftward_1, cell_leftward_2, color_not_pressed, -1, 1 ,0);
    cv::rectangle(img, cell_rightward_1, cell_rightward_2, color_not_pressed, -1, 1 ,0);

    cv::Scalar color_pressed = cv::Scalar(0, 0, 255);
    switch(direction)
    {
        case 0:
            cv::rectangle(img, cell_forward_1, cell_forward_2, color_pressed, -1, 1 ,0);
            break;
        case 1:
            cv::rectangle(img, cell_backward_1, cell_backward_2, color_pressed, -1, 1 ,0);
            break;
        case 2:
            cv::rectangle(img, cell_leftward_1, cell_leftward_2, color_pressed, -1, 1 ,0);
            break;
        case 3:
            cv::rectangle(img, cell_rightward_1, cell_rightward_2, color_pressed, -1, 1 ,0);
            break;
        default:
            break;
    }

    boxes.clear();
}

void callbackRGB(const sensor_msgs::ImageConstPtr& rgb_msg)
{
    // Read from sensor msg
    cv_bridge::CvImagePtr rgb_ptr;
    try
    {
        rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e1)
    {
        ROS_ERROR("cv_bridge rgb exception: %s", e1.what());
        return;
    }
    cv::Mat rgb_img = rgb_ptr->image;
    cv::resize(rgb_img, rgb_img, cv::Size(IMGWIDTH,IMGHEIGHT));


    add_infos(rgb_img);

    cv::imshow("Monitor", rgb_img);
    cv::waitKey(1);
}

void callbackDepth(const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e2)
    {
        ROS_ERROR("cv_bridge depth exception: %s", e2.what());
        return;
    }
    cv::Mat depth_img = depth_ptr->image;
    depth_img_global = depth_img;

    if(mode == 0)  // Do not show in RGB mode
        return;

    /// Transform to a Uint8 image
    int nr = depth_img.rows;
    int nc = depth_img.cols;
    cv::Mat depth_uint(nr, nc, CV_8UC1, cv::Scalar::all(0));

    for(int i=0; i<nr ;i++)
    {
        float *inDepth = depth_img.ptr<float>(i); // float
        uchar* inDepth_uint = depth_uint.ptr<uchar>(i);

        for(int j=0; j<nc; j++)
        {
            // cout<< " " << inDepth[j];
            if (inDepth[j] > MAX_DEPTH || inDepth[j] != inDepth[j]) {
                inDepth_uint[j] = 0;
            }
            else {
                inDepth_uint[j] = (uchar)floor(inDepth[j] * 56.0 * 0.001); // 56 = 256/4.5
            }
        }
    }


    // cv::imshow("depth", depth_uint);
    // cv::waitKey(1);

    cv::Mat depth_3_channels = cv::Mat(nr, nc, CV_8UC3);
    cv::cvtColor(depth_uint, depth_3_channels, CV_GRAY2BGR);

    cv::resize(depth_3_channels, depth_3_channels, cv::Size(IMGWIDTH,IMGHEIGHT));


    add_infos(depth_3_channels);

    cv::imshow("Monitor", depth_3_channels);
    cv::waitKey(1);
}

void callbackCommand(const std_msgs::Float64MultiArray& cmd_msg)
{
    if(cmd_msg.data[0] == 1.0){
        direction = 0;
    }
    else if(cmd_msg.data[1] == 1.0){
        direction = 1;
    }
    else if(cmd_msg.data[2] == 1.0){
        direction = 2;
    }
    else if(cmd_msg.data[3] == 1.0)
    {
        direction = 3;
    }
    else{
        direction = -1;
    }

}

void callbackObjects(const darknet_ros_msgs::BoundingBoxes& objects)
{
    boxes.clear();
    float minimum_dist = 10000.0;

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

            int mid_x = (objects.bounding_boxes[m].xmin + objects.bounding_boxes[m].xmax) / 2;
            int mid_y = (objects.bounding_boxes[m].ymin + objects.bounding_boxes[m].ymax) / 2;

            float object_z = depth_img_global.at<float>(mid_x, mid_y);
            if(object_z != object_z) object_z = MAX_DEPTH; // NAN issue

            int rect_length_half = 10;
            for(int i = -1; i < 2; i ++)  // Find nearest point among center 10 points. Character "Tian" corners
            {
                int offset_x = i * rect_length_half;

                for(int n = -1; n < 2; n++)
                {
                    int offset_y = n * rect_length_half;

                    int new_x = mid_x + offset_x;
                    int new_y = mid_y + offset_y;

                    if(new_x < 1 || new_x > IMGWIDTH - 2) continue;
                    if(new_y < 1 || new_y > IMGHEIGHT - 2) continue;

                    float sampled_z = depth_img_global.at<float>(new_x, new_y);
                    if(sampled_z == sampled_z; && sampled_z < object_z){
                        object_z = sampled_z;
                        mid_x = new_x;
                        mid_y = new_y;
                    }
                }

            }
            object_z = object_z /1000.f;  //mm to m

            static const float camera_fx = 555.f; // !!!! CHG NOTE
            static const float camera_fy = 555.f;

            float z_div_camera_fx = object_z /camera_fx;
            float z_div_camera_fy = object_z /camera_fy;

            float object_x = (mid_x - IMGWIDTH / 2) * z_div_camera_fx;
            float object_y = (mid_y - IMGHEIGHT / 2) * z_div_camera_fy;

            float plane_dist = sqrt(object_x*object_x + object_z*object_z);
            if(plane_dist < minimum_dist){
                minimum_dist = plane_dist;
            }

        }
        else if (objects.bounding_boxes[m].Class == "cat")
            label = 6;
        else if (objects.bounding_boxes[m].Class == "dog")
            label = 6;
        else if (objects.bounding_boxes[m].Class == "laptop")
            label = 5;
        else if (objects.bounding_boxes[m].Class == "bed")
            label = 5;
        else if (objects.bounding_boxes[m].Class == "tvmonitor")
            label = 5;
        else if (objects.bounding_boxes[m].Class == "chair")
            label = 5;
        else if (objects.bounding_boxes[m].Class == "diningtable")
            label = 5;
        else if (objects.bounding_boxes[m].Class == "sofa")
            label = 5;
        else
            label = 3;

        cv::Scalar label_color;
        if(label == 7)
            label_color = cv::Scalar(48, 48, 255);
        else if(label == 6)
            label_color = cv::Scalar(0, 165, 255);
        else if(label == 5)
            label_color = cv::Scalar(0, 255, 255);
        else
            label_color = cv::Scalar(152, 251, 152);

        if(label >= 3)
        {
            cv::Point p1 = cv::Point(objects.bounding_boxes[m].xmin, objects.bounding_boxes[m].ymin);
            cv::Point p2 = cv::Point(objects.bounding_boxes[m].xmax, objects.bounding_boxes[m].ymax);
            SemanticBox object = {objects.bounding_boxes[m].Class, p1, p2, label_color};
            boxes.push_back(object);
        }
    }

    if(minimum_dist < 10000.0)
    {
        std_msgs::Float64 dist_to_pub;
        dist_to_pub.data = minimum_dist;
        person_dist_pub.publish(dist_to_pub);
    }

}

void callbackOutput(const geometry_msgs::Twist& output)
{
    linear_v = output.linear.x;
    angular_v = output.angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display");
    ros::NodeHandle nh;

    /// Check arguments to choose display mode
    if(argc != 2){
        cout << "*** Arguments number Error. Please enter one mode." << endl;
        cout << "Candidates are: rgb, depth, depth_semantic" << endl;
        return 0;
    }

    person_dist_pub = nh.advertise<std_msgs::Float64>("/person_dist", 1);

    ros::Subscriber direction_sub = nh.subscribe("/radar/direction", 1, callbackCommand);
    ros::Subscriber output_sub = nh.subscribe("/smoother_cmd_vel", 1, callbackOutput);

    ros::Subscriber image_sub, objects_sub;

    /// Display according to different modes
    string input_mode = argv[1];
    if(input_mode == "rgb")
    {
        image_sub = nh.subscribe("/kinect2/qhd/image_color", 1, callbackRGB);
        image_sub = nh.subscribe("/kinect2/sd/image_depth", 1, callbackDepth);
        objects_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, callbackObjects);
        mode = 0;
    }
    else if(input_mode == "depth")
    {
        image_sub = nh.subscribe("/kinect2/sd/image_depth", 1, callbackDepth);
        objects_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, callbackObjects);
        mode = 1;
    }
    else if(input_mode == "depth_semantic")
    {
        image_sub = nh.subscribe("/kinect2/sd/image_depth", 1, callbackDepth);
        objects_sub = nh.subscribe("/darknet_ros/bounding_boxes", 2, callbackObjects);
        mode = 2;
    }
    else
    {
        cout << "*** Error: " << argv[1] <<" is not a valid argument." << endl;
        cout << "Candidates are: rgb, depth, depth_semantic" << endl;
        return 0;
    }

    cv::namedWindow("Monitor", CV_WINDOW_NORMAL);
    cvMoveWindow( "Monitor", 100, 100);

    ros::spin();

    return 0;
}

