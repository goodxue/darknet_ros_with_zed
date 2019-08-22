/*
* @file realsense_photo.cpp
* @brief Using realsense to capture image and save it as .jpg
* @author Weipeng.Xue  <goodxue@gmail.com>
* @version 1.0
*
************************************************
*
* Copyright (c) 2019 Weipeng.Xue.
*
* All rights reserved.
*
************************************************
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.h"     
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;


int rate  = 30;
Mat globalImage(Size(640,480),CV_8UC3);
int chess_num =0;
std::string path = "/home/xue/Documents";

void imageCallback1(const sensor_msgs::ImageConstPtr& tem_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(tem_msg, "bgr8");

        Mat chess = cv_ptr->image;


        cv::imshow("color", chess);
        if(waitKey(10) == 's')
        {
            ROS_INFO_STREAM("Saved : "<<path<<""<<chess_num++<<".jpg");

            imwrite(path,chess);
        }


    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_savepicture");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub1 = it.subscribe("/camera/color/image_raw",1,imageCallback1);

    ros::Rate loop_rate(rate);
    while (nh.ok()) {
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
