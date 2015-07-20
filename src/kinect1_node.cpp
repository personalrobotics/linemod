#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
image_transport::Publisher *rgb_pub = NULL;
ros::Publisher *rgb_camera_pub = NULL;
image_transport::Publisher *depth_pub = NULL;
ros::Publisher *depth_camera_pub = NULL;

void RGBImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    // assert the resolution is 960x540
    assert(cv_ptr->image.rows == 540 && cv_ptr->image.cols == 960);
    // new resolution = 640x480, cropped around centre
    cv::Rect roi((960-640)/2., (540-480)/2.,  640, 480);
    cv_ptr->image = cv_ptr->image(roi);
    rgb_pub->publish(cv_ptr->toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

// Callback for RGB camera info
void RGBCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  sensor_msgs::CameraInfo ci = *camera_info;
  ci.height = 480;
  ci.width = 640;
  // focal length remains same, offset reduces by (960-640)/2 and (540-480)/2
  ci.K[2] -= (960-640)/2.;
  ci.K[5] -= (540-480)/2.;

  ci.P[2] -= (960-640)/2.;
  ci.P[6] -= (540-480)/2.;

  rgb_camera_pub->publish(ci);
}

// Callback for Depth camera info
void DepthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  sensor_msgs::CameraInfo ci = *camera_info;
  ci.height = 480;
  ci.width = 640;
  // focal length remains same, offset reduces by (960-640)/2 and (540-480)/2
  ci.K[2] -= (960-640)/2.;
  ci.K[5] -= (540-480)/2.;

  ci.P[2] -= (960-640)/2.;
  ci.P[6] -= (540-480)/2.;

  depth_camera_pub->publish(ci);
}


void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    // assert the resolution is 960x540
    assert(cv_ptr->image.rows == 540 && cv_ptr->image.cols == 960);
    // new resolution = 640x480, cropped around centre
    cv::Rect roi((960-640)/2., (540-480)/2.,  640, 480);
    cv_ptr->image = cv_ptr->image(roi);
    depth_pub->publish(cv_ptr->toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect1_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // rgb
  image_transport::Subscriber rgb_sub = it.subscribe(
    "/head/kinect2/qhd/image_color_rect", 1, RGBImageCallback);
  image_transport::Publisher rgb_pub_ = it.advertise("kinect1_rgb_image", 1);
  rgb_pub = &rgb_pub_;

  ros::Subscriber rgb_camera_sub = nh.subscribe(
    "/head/kinect2/qhd/camera_info", 10, &RGBCameraInfoCallback);
  ros::Publisher rgb_camera_pub_ = nh.advertise<sensor_msgs::CameraInfo>(
    "kinect1_rgb_camera_info", 10);
  rgb_camera_pub = &rgb_camera_pub_;

  // depth
  image_transport::Subscriber depth_sub = it.subscribe(
    "/head/kinect2/qhd/image_depth_rect", 1, DepthImageCallback);
  image_transport::Publisher depth_pub_ = it.advertise("kinect1_depth_image", 1);
  depth_pub = &depth_pub_;

  ros::Subscriber depth_camera_sub = nh.subscribe(
    "/head/kinect2/qhd/camera_info", 10, &DepthCameraInfoCallback);
  ros::Publisher depth_camera_pub_ = nh.advertise<sensor_msgs::CameraInfo>(
    "kinect1_depth_camera_info", 10);
  depth_camera_pub = &depth_camera_pub_;

  ros::spin();
  return 0;
}