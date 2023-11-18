#include <depthimage_to_laserscan_hsk/DepthimageToLaserscan.h>
using namespace depthimage_to_laserscan;

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
                                             float height_max)
{
    inited_ = false;
    angle_max_ = angle_max;
    angle_min_ = angle_min;
    output_frame_id_ = output_frame_id;
    scan_time_ = scan_time;
    range_max_ = range_max;
    range_min_ = range_min;
    scan_height_ = scan_height;
    group_size_ = group_size;
    scan_center_ = scan_center;
    height_min_ = height_min;
    height_max_ = height_max;
}

DepthimageToLaserscan::~DepthimageToLaserscan()
{
}

double DepthimageToLaserscan::magnitude_of_ray(const cv::Point3d ray) const
{
    return sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
}

double DepthimageToLaserscan::angle_of_rays(const cv::Point3d ray1, cv::Point3d ray2) const
{
    const double dot_product = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
    const double manitude1 = magnitude_of_ray(ray1);
    const double manitude2 = magnitude_of_ray(ray2);
    return acos(dot_product / (manitude1 * manitude2));
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

sensor_msgs::LaserScanPtr DepthimageToLaserscan::convert_msg(const sensor_msgs::ImageConstPtr &depth_msg, const cv::Mat &image,
                                                             const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    if (!inited_)
    {
        cam_model_.fromCameraInfo(info_msg);

        cv::Point2d raw_pixel_left(0, cam_model_.cy());
        cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
        cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

        cv::Point2d raw_pixel_right(depth_msg->width - 1, cam_model_.cy());
        cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
        cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

        cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
        cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
        cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

        angle_max_ = angle_of_rays(center_ray, left_ray);
        angle_min_ = -angle_of_rays(center_ray, right_ray);

        inited_ = true;
    }

    sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
    scan_msg->header = depth_msg->header;
    if (output_frame_id_.length() > 0)
    {
        scan_msg->header.frame_id = output_frame_id_;
    }

    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = (angle_max_ - angle_min_) / (depth_msg->width - 1);
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_max = range_max_;
    scan_msg->range_min = range_min_;

    if (0.5 * scan_height_ * group_size_ > cam_model_.cy() || 0.5 * scan_height_ * group_size_ > depth_msg->height - cam_model_.cy())
    {
        std::stringstream ss;
        ss << "scan_height (" << scan_height_ << ")pixels is too large for the image height.";
        throw std::runtime_error(ss.str());
    }

    const uint32_t ranges_size = depth_msg->width;
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        convert<uint16_t>(image, scan_msg);
    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        convert<float>(image, scan_msg);
    }
    else
    {
        std::stringstream ss;
        ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
        throw std::runtime_error(ss.str());
    }

    return scan_msg;
}

template <typename T>
void DepthimageToLaserscan::convert(const cv::Mat &depth_msg, const sensor_msgs::LaserScanPtr &scan_msg) const
{
    const float center_x = cam_model_.cx();
    const float center_y = cam_model_.cy();

    const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters(T(1));
    const float costant_x = unit_scaling / cam_model_.fx();
    const float costant_y = unit_scaling / cam_model_.fy();

    for (size_t u = 0; u < depth_msg.cols; u++)
    {
        const double angle = -atan2f((float)(u - center, cam_model.fx()));
        const int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;

        for (size_t v = scan_center_ - 0.5 * group_size_ * scan_height_; v < scan_center_ + 0.5 * group_size_ * scan_height_; v += group_size_)
        {
            double min_d = scan_msg->range_max;
            cv::Point point;
            for (size_t i = 0; i < group_size_; i++)
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

                if (usePoint(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max) && usePoint(y, height_min_, height_max_))
                {
                    scan_msg->ranges[index] = r;
                }
            }
        }
    }
}