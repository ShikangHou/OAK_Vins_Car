#include <depthimage_to_laserscan_hsk/DepthimageToLaserscanROS.h>

using namespace depthimage_to_laserscan;

DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it_(nh), nh_(nh), pnh_(pnh)
{
    int badparam = 0;
    std::string output_frame_id;
    float scan_time, angle_min, angle_max, range_max, range_min, height_min, height_max;
    int scan_height, group_size, scan_center;

    badparam += !pnh_.getParam("output_fream_id", output_frame_id);
    badparam += !pnh_.getParam("scan_time", scan_time);
    badparam += !pnh_.getParam("range_min", range_min);
    badparam += !pnh_.getParam("range_max", range_max);
    badparam += !pnh_.getParam("scan_height", scan_height);
    badparam += !pnh_.getParam("group_size", group_size);
    badparam += !pnh_.getParam("scan_center", scan_center);
    badparam += !pnh_.getParam("height_min", height_min);
    badparam += !pnh_.getParam("height_max", height_max);

    if (badparam > 0)
    {
        std::cout << " Bad parameters -> " << badparam << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }

    it_pub_ = it_.advertise("/it_cam",2);
    dtl_ = new DepthimageToLaserscan(output_frame_id, scan_time, range_min, range_max, scan_height, group_size, scan_center, height_min, height_max);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("/laserscan", 10, std::bind(&DepthImageToLaserScanROS::connectCb, this, std::placeholders::_1), std::bind(&DepthImageToLaserScanROS::disconnectCb, this, std::placeholders::_1));
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher &pub)
{
    std::unique_lock<std::mutex> connect_lock(connect_mutex_);
    if (!sub_ && pub_.getNumSubscribers() > 0)
    {
        ROS_DEBUG("Connecting to depth topic");

        sub_ = it_.subscribeCamera("image", 10, std::bind(&DepthImageToLaserScanROS::depthCb, this, std::placeholders::_1, std::placeholders::_2));
    }
    connect_lock.unlock();
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher &pub)
{
    std::unique_lock<std::mutex> disconnect_lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
        sub_.shutdown();
    }
    disconnect_lock.unlock();
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                                       const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), depth_msg->encoding, image).toImageMsg();
        it_pub_.publish(depth_msg);
        sensor_msgs::LaserScanPtr scan_msg = dtl_->convert_msg(depth_msg, image, info_msg);
        pub_.publish(scan_msg);
        
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Could not convert depth image to laserscan: %s", e.what());
    }
}