#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> // for converting the command line parameter to integer  

    int main(int argc, char** argv)  
    {  
      // Check if video source has been passed as a parameter  
      if(argv[1] == NULL)   
        {  
            // ROS_INFO("argv[1]=NULL\n");  
            // return 1;  
            char ved_n[] = "video.mp4";
            argv[1] = ved_n;
        }  

      ros::init(argc, argv, "vedio_publisher");  
      ros::NodeHandle nh;  
      image_transport::ImageTransport it(nh);  
      image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);   

      cv::VideoCapture cap(argv[1]);  
      // Check if video device can be opened with the given index  
      if(!cap.isOpened())   
      {  
          ROS_INFO("can not opencv video device\n");  
          return 1;  
      }  
      cv::Mat frame; 
      sensor_msgs::ImagePtr msg;  

      ros::Rate loop_rate(5);  
      while (nh.ok()) {  
        cap >> frame;  
        // Check if grabbed frame is actually full with some content  
        if(!frame.empty()) {  
          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
          pub.publish(msg);  
          //cv::Wait(1);  
        }  

        ros::spinOnce();  
      }
    }