#include <depthimage_to_laserscan_hsk/DepthimageToLaserscan.h>
using namespace depthimage_to_laserscan;

DepthimageToLaserscan::DepthimageToLaserscan(/* args */)
{
}

DepthimageToLaserscan::~DepthimageToLaserscan()
{
}

bool DepthimageToLaserscan::usePoint(const float new_value, const float old_value, const float range_min, const float range_max) const
{
    const bool new_finite = std::isfinite(new_value);
    const bool old_finite = std::isfinite(old_value);

    if (!new_finite && !old_finite) // 两个都不是有限值，可能是inf或者nan
    {
        if (std::isnan(new_value)) // 只要新值不是nan，就有效
        {
            return true;
        }
        else
            return false;
    }

    const bool range_check = (new_value >= range_min) && (new_value <= range_min);
    if (!range_check) // new value is not in range
    {
        return false;
    }

    if (!old_finite) // new value is in range and finite,use it
    {
        return true;
    }

    const bool shorter_check = new_value < old_value;
    return shorter_check;
}

bool DepthimageToLaserscan::usePoint(const float new_value, const float range_min, const float range_max) const
{
    if (!std::isfinite(new_value))
    {
        return false;
    }

    const bool ranger_checker = (new_value <= range_max) && (new_value >= range_min);
    return ranger_checker;
}

template <typename T>
void DepthimageToLaserscan::convert(const cv::Mat &depth_msg, const image_geometry::PinholeCameraModel &cam_model, const sensor_msgs::LaserScanPtr &scan_msg,
                                    const int &scan_center, const int group_size, const int &scan_height, const float height_min, const float height_max) const
{
    const float center_x = cam_model.cx();
    const float center_y = cam_model.cy();

    const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters(T(1));
    const float costant_x = unit_scaling / cam_model.fx();
    const float costant_y = unit_scaling / cam_model.fy();

    for (size_t u = 0; u < depth_msg.cols; u++)
    {
        const double angle = -atan2f((float)(u - center, cam_model.fx()));
        const int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;

        for (size_t v = scan_center - group_size * scan_height; v < scan_center + group_size * scan_height; v += group_size)
        {
            double min_d = scan_msg->range_max;
            cv::Point point;
            for (size_t i = 0; i < group_size; i++)
            {
                if (depthimage_to_laserscan::DepthTraits<T>::valid(depth))
                {
                    T depth = depth_msg.at<T>(v + i, u);

                    if (depth < min_d)
                    {
                        min_d = depth;
                        point = cv::Point(u, v + i);
                    }
                }
            }

            if (min_d != scan_msg->range_max)
            {
                double x = (point.x - center_x) * min_d * costant_x;
                double y = (point.y - center_y) * min_d * costant_y;
                double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);
                double r = hypot(x, z);

                if ()
                {
                }
            }
        }
    }
}