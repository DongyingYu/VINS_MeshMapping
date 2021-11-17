#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include "parameters.h"


using namespace std;
using namespace cv;
 
// ros::Publisher pub_ply_map;

int main (int argc, char **argv)
{
    ros::init (argc, argv, "loadply");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(nh);
    //ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    ros::Publisher pub_ply_map;
    //PointXYZRGBA
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    // std::cout << "address :" << addressfile << std::endl;
    ROS_INFO_STREAM ("address " << addressfile);
    if (pcl::io::loadPLYFile<pcl::PointXYZRGBA>(addressfile, cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}

    sensor_msgs::PointCloud2 output;
    
    // Fill in the cloud data
    // cloud.width  = 100;
    // cloud.height = 1;
    // cloud.points.resize(cloud.width * cloud.height);
 
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    // }
 
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "mission";
 
    pub_ply_map = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    //循环每秒发送一次
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        // ROS_INFO_STREAM ("address " << addressfile);
        pub_ply_map.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
 
    return 0;
}
