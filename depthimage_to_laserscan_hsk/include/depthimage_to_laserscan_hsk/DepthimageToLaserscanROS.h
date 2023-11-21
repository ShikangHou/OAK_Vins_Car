#ifndef DEPTHIMAGE_TO_LASERSCANROS_H_
#define DEPTHIMAGE_TO_LASERSCANROS_H_

#include <ros/ros.h>
#include <depthimage_to_laserscan_hsk/DepthimageToLaserscan.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace depthimage_to_laserscan
{
  class DepthImageToLaserScanROS
  {

  private:
    DepthimageToLaserscan *dtl_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher it_pub_;
    ros::Publisher pub_;
    
    std::mutex connect_mutex_;

  public:
    DepthImageToLaserScanROS(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void connectCb(const ros::SingleSubscriberPublisher &pub);
    void disconnectCb(const ros::SingleSubscriberPublisher &pub);
    void depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                                           const sensor_msgs::CameraInfoConstPtr &info_msg);
  };

}

#endif
