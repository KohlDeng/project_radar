#include <iostream>
#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;



class velodyne_cloud
{
private:
	ros::NodeHandle n;
	sensor_msgs::PointCloud cloud;
	ros::Publisher pub;
	ros::Subscriber sub;

public:
	velodyne_cloud(ros::NodeHandle nh);

	void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		sensor_msgs::PointCloud2 raw;
		raw = *msg;
		
		// pcl::fromROSMsg(msg,rawCloud);

		bool flag = sensor_msgs::convertPointCloud2ToPointCloud(raw,cloud);

		if (flag)
		{
			pub.publish(cloud);
		}
	}

	void pclcallback(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*msg,pcl_pc2);
		pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromPCLPointCloud2(pcl_pc2,*tempCloud);

		// pcl::PointCloud<pcl::PointXYZI> pcl_pc;
		// pcl::fromROSMsg(*msg, pcl_pc);
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*tempCloud,output);
		pub.publish(output);

	}
};

velodyne_cloud::velodyne_cloud(ros::NodeHandle nh):n(nh)
{
	pub=nh.advertise<sensor_msgs::PointCloud2>("velodyne/cloud",2);
	sub = nh.subscribe("velodyne/left_front",2,&velodyne_cloud::pclcallback,this);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"velodyne_pub_node");
	ros::NodeHandle nh;
	velodyne_cloud llc(nh);

	ros::spin();
	return 0;

}