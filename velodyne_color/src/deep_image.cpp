#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "ros/ros.h"

//learning

int main(int argc,char **argv)
{
    ros::init(argc,argv,"deep_image");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud2>("deep/pointcloud",2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);//声明点云
 
    for(float y=-0.5f;y<=0.5f;y+=0.01f)
    {
        for (float z=-0.5f; z<=0.5f; z+=0.01f) 
        {
            pcl::PointXYZ point;//声明pcl的一个点
            //赋值点云里点 的 xyz坐标
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud->points.push_back(point);
        }
    }
    //赋值点云的宽度与高度
    pointCloud->width = (uint32_t) pointCloud->points.size();
    pointCloud->height = 1;

    //pcl::visualization::CloudViewer viewer1("Cloud Viewer");
    sensor_msgs::PointCloud2 msg_c;
    pcl::toROSMsg(*pointCloud, msg_c);
    msg_c.header.frame_id = "map";

    ros::Rate rate_hz(20);
    while(1)
    {
    	//viewer1.showCloud(pointCloud);
        pub.publish(msg_c);
        rate_hz.sleep();
        ros::spinOnce();
	}
	return 0;
}