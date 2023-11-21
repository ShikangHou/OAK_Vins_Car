#include <ros/ros.h>
#include <depthimage_to_laserscan_hsk/DepthimageToLaserscanROS.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"Depthimage2Laserscan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    depthimage_to_laserscan::DepthImageToLaserScanROS dtl_ros(nh,pnh);
    
    ros::spin();
    return 0;
}
