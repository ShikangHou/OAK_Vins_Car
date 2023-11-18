#ifndef DEPTHIMAGE_TO_LASERSCAN_H_
#define DEPTHIMAGE_TO_LASERSCAN_H_

#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include "depthimage_to_laserscan_hsk/DepthTraits.h"

namespace depthimage_to_laserscan
{
    class DepthimageToLaserscan
    {
    private:
        image_geometry::PinholeCameraModel cam_model_;
        bool inited_;
        float angle_max_;
        float angle_min_;
        std::string output_frame_id_;
        float scan_time_;
        float range_max_;
        float range_min_;
        int scan_height_;
        int group_size_;
        int scan_center_;
        float height_min_;
        float height_max_;

    public:
        DepthimageToLaserscan::DepthimageToLaserscan(std::string output_frame_id,
                                             float angle_min,
                                             float angle_max,
                                             float scan_time,
                                             float range_max,
                                             float range_min,
                                             int scan_height,
                                             int group_size,
                                             int &scan_center,
                                             float height_min,
                                             float height_max);
        ~DepthimageToLaserscan();

        bool usePoint(const float new_value, const float old_value, const float range_min, const float range_max) const;
        bool usePoint(const float value, const float range_min, const float range_max) const;
        double magnitude_of_ray(const cv::Point3d ray) const;
        double angle_of_rays(const cv::Point3d ray1, cv::Point3d ray2) const;
        sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr &depth_msg, const cv::Mat &image,
                                              const sensor_msgs::CameraInfoConstPtr &info_msg);

        template <typename T>
        void convert(const cv::Mat &depth_msg, const sensor_msgs::LaserScanPtr &scan_msg) const;
    };

};

#endif
