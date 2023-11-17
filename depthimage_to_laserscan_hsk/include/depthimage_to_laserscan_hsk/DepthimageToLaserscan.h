#ifndef DEPTHIMAGE_TO_LASERSCAN_H_
#define DEPTHIMAGE_TO_LASERSCAN_H_

#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "depthimage_to_laserscan_hsk/DepthTraits.h"

namespace depthimage_to_laserscan
{
    class DepthimageToLaserscan
    {
    private:
        /* data */
    public:
        DepthimageToLaserscan(/* args */);
        ~DepthimageToLaserscan();

        bool usePoint(const float new_value,const float old_value,const float range_min,const float range_max) const;
        bool usePoint(const float value,const float range_min,const float range_max) const;


        template <typename T>
        void convert(const cv::Mat &depth_msg, const image_geometry::PinholeCameraModel &cam_model, const sensor_msgs::LaserScanPtr &scan_msg,
                     const int &scan_center, const int group_size, const int &scan_height, const float height_min, const float height_max) const;
    };

};

#endif
