#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sl_zed/Camera.hpp>
#include <yolo_test/BoundingBoxesZed.h>
#include <yolo_test/BoundingBox.h>
//#include <cv_bridge/cv_bridge.h>

class yolo_ros{
public:
  yolo_ros();
  ~yolo_ros();
  void close();
private:
  // bool refresh_sl_measure_cld();
  void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  // sl::Camera zed;
  //sl::Mat point_cloud;
  // sl::RuntimeParameters runtime_parameters;
  ros::NodeHandle nh;
  ros::Subscriber yolo_sub;
  ros::Publisher objxyz_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  sensor_msgs::ImageConstPtr point_cloud;     //使用其格式指针保存返回的msg
};


yolo_ros::yolo_ros():it(nh){
  // sl::InitParameters init_params;
  // init_params.depth_mode = sl::DEPTH_MODE_QUALITY; // Use PERFORMANCE depth mode
  // init_params.coordinate_units = sl::UNIT_MILLIMETER; // Use millimeter units (for depth measurements)
  // //init_params.depth_minimum_distance = 50;

  // sl::ERROR_CODE err = zed.open(init_params);
  // if (err != sl::SUCCESS) {
  //     exit(-1);
  // }
  // runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;  //设置运行参数为标准
  image_sub = it.subscribe("/zed/depth/depth_registered", 1,&yolo_ros::imageCallback,this);
  yolo_sub = nh.subscribe("/darknet_ros/bounding_boxes",1,&yolo_ros::yolo_callback,this);
  objxyz_pub = nh.advertise<yolo_test::BoundingBoxesZed>(std::string("/obj3d"),1);
}


yolo_ros::~yolo_ros(){
  // zed.close();
}


void yolo_ros::close() {
  this->~yolo_ros();
}



void yolo_ros::imageCallback(const sensor_msgs::ImageConstPtr& msg){
  point_cloud = msg;
}



// bool yolo_ros::refresh_sl_measure_cld() {
//   if(zed.retrieveMeasure(point_cloud,sl::MEASURE_XYZ)) {
//     return true; 
//   }
//   else {
//     ROS_ERROR_STREAM("can't retrieve point cloud image from zed!");
//   }
// }



void yolo_ros::yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)  //费了老鼻子劲写这个函数了
{
    // if(!refresh_sl_measure_cld()){
    //   if(!point_cloud.isInit()) {
    //     ROS_ERROR_STREAM("sl::Mat hasn't been allocated!!!");
    //     return;
    //   }
    // }//更新mat
    if(!point_cloud){
      return ;
    }
    yolo_test::BoundingBoxesZed pub_msg;
    float depth_temp;
    for (auto &cur_box : msg->bounding_boxes) { 

      float* depths = (float*)(&point_cloud->data[0]);

      // Image coordinates of the center pixel
      int u = (cur_box.xmax+cur_box.xmin) / 2;
      int v = (cur_box.ymax+cur_box.ymin) / 2;

      // Linear index of the center pixel
      int centerIdx = u + point_cloud->width * v;

      // Output the measure
      depth_temp = depths[centerIdx];

      yolo_test::BoundingBox boundingbox_temp;
      boundingbox_temp.Class = cur_box.Class;
      boundingbox_temp.probability = cur_box.probability;
      boundingbox_temp.xmax = cur_box.xmax;
      boundingbox_temp.xmin = cur_box.xmin;
      boundingbox_temp.ymax = cur_box.ymax;
      boundingbox_temp.ymin = cur_box.ymin;
      pub_msg.bounding_boxes.push_back(boundingbox_temp);
      pub_msg.depth_array.push_back(depth_temp);
      pub_msg.header = msg->header;
      pub_msg.image_header = msg->image_header;
  }
      objxyz_pub.publish(pub_msg);
}

int main(int argc, char** argv)  
{  
  // Check if video source has been passed as a parameter  

  ros::init(argc, argv, "vedio_publisher");  
  ros::NodeHandle nh;
  // Check if video device can be opened with the given index  
  yolo_ros yr;

  ros::Rate loop_rate(20);  
  while (nh.ok()) {  

    loop_rate.sleep();
    ros::spinOnce();  
  }
}