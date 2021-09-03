
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <project_radar/common.h>

using namespace std;

#define Hmax 1080
#define Wmax 1920
#define H Hmax
#define W Wmax

// 使用pcl::PointXYZI结构体
typedef pcl::PointXYZI PointType;

//全局变量都能访问，图像回调中写，点云回调中读
 cv::Mat imageMat;


const int threshold_lidar=30000;
float max_depth = 10;
float min_depth = 1;

// 内外参
cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);	   // 畸变向量
cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
cv::Mat intrinsicMat(3, 4, cv::DataType<double>::type);	   // 内参3*4的投影矩阵，最后一列是三个零
cv::Mat extrinsicMat_RT(4, 4, cv::DataType<double>::type); // 外参旋转矩阵3*3和平移向量3*1

string intrinsic_path, extrinsic_path;

void getParameters()
{
	cout<<"get the parameters from the launch file"<<endl;

	if(!ros::param::get("intrinsic_path",intrinsic_path)){
		cout<<"can not get the value of intrinsic_path"<<endl;
		exit(1);
	}
	if(!ros::param::get("extrinsic_path",extrinsic_path)){
		cout<<"can not get the value of extrinsic_path"<<endl;
		exit(1);
	}
}

void CalibrationData(void)
{
	vector<float> intr;
	getIntrinsic(intrinsic_path, intr);
	vector<float> distortion;
	getDistortion(intrinsic_path,distortion);
	vector<float> extrinsic;
	getExtrinsic(extrinsic_path,extrinsic);

	intrinsic.at<double>(0, 0) = intr[0];
	intrinsic.at<double>(0, 2) = intr[2];
	intrinsic.at<double>(1, 1) = intr[4];
	intrinsic.at<double>(1, 2) = intr[5];

	intrinsicMat = cv::Mat::zeros(3,4,CV_64F);
    intrinsicMat.at<double>(0, 0) = intr[0];
	intrinsicMat.at<double>(0, 2) = intr[2];
	intrinsicMat.at<double>(1, 1) = intr[4];
	intrinsicMat.at<double>(1, 2) = intr[5];
	intrinsicMat.at<double>(2, 2) = 1.000000e+00;

	distCoeffs = cv::Mat::zeros(5,1,CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];
	
    extrinsicMat_RT = cv::Mat::zeros(4,4,CV_64F);
	extrinsicMat_RT.at<double>(0, 0) = extrinsic[0];
	extrinsicMat_RT.at<double>(0, 1) = extrinsic[1];
	extrinsicMat_RT.at<double>(0, 2) = extrinsic[2];
	extrinsicMat_RT.at<double>(0, 3) = extrinsic[3];
	extrinsicMat_RT.at<double>(1, 0) = extrinsic[4];
	extrinsicMat_RT.at<double>(1, 1) = extrinsic[5];
	extrinsicMat_RT.at<double>(1, 2) = extrinsic[6];
	extrinsicMat_RT.at<double>(1, 3) = extrinsic[7];
	extrinsicMat_RT.at<double>(2, 0) = extrinsic[8];
	extrinsicMat_RT.at<double>(2, 1) = extrinsic[9];
	extrinsicMat_RT.at<double>(2, 2) = extrinsic[10];
	extrinsicMat_RT.at<double>(2, 3) = extrinsic[11];
	extrinsicMat_RT.at<double>(3, 0) = 0.0;
	extrinsicMat_RT.at<double>(3, 1) = 0.0;
	extrinsicMat_RT.at<double>(3, 2) = 0.0;
	extrinsicMat_RT.at<double>(3, 3) = 1.0;

}

void printMat()
{
	cout<<"=============extrinsic==============="<<endl;
	cout<<extrinsicMat_RT.at<double>(0, 0)<<"\t"; 
	cout<<extrinsicMat_RT.at<double>(0, 1)<<"\t";
	cout<<extrinsicMat_RT.at<double>(0, 2)<<"\t";
	cout<<extrinsicMat_RT.at<double>(0, 3)<<endl;
	cout<<extrinsicMat_RT.at<double>(1, 0) <<"\t";
	cout<<extrinsicMat_RT.at<double>(1, 1) <<"\t";
	cout<<extrinsicMat_RT.at<double>(1, 2) <<"\t";
	cout<<extrinsicMat_RT.at<double>(1, 3) <<endl;
	cout<<extrinsicMat_RT.at<double>(2, 0) <<"\t";
	cout<<extrinsicMat_RT.at<double>(2, 1) <<"\t";
	cout<<extrinsicMat_RT.at<double>(2, 2) <<"\t";
	cout<<extrinsicMat_RT.at<double>(2, 3) <<endl;
	cout<<extrinsicMat_RT.at<double>(3, 0) <<"\t";
	cout<<extrinsicMat_RT.at<double>(3, 1) <<"\t";
	cout<<extrinsicMat_RT.at<double>(3, 2) <<"\t";
	cout<<extrinsicMat_RT.at<double>(3, 3) <<endl;

	cout<<"==================intrinsic==========="<<endl;
	cout<<intrinsicMat.at<double>(0, 0) <<"\t"; 
	cout<<intrinsicMat.at<double>(0, 1) <<"\t"; 
	cout<<intrinsicMat.at<double>(0, 2) <<"\t"; 
	cout<<intrinsicMat.at<double>(0, 3) <<endl;
	cout<<intrinsicMat.at<double>(1, 0) <<"\t"; 
	cout<<intrinsicMat.at<double>(1, 1) <<"\t"; 
	cout<<intrinsicMat.at<double>(1, 2) <<"\t"; 
	cout<<intrinsicMat.at<double>(1, 3) <<endl;
	cout<<intrinsicMat.at<double>(2, 0) <<"\t"; 
	cout<<intrinsicMat.at<double>(2, 1) <<"\t"; 
	cout<<intrinsicMat.at<double>(2, 2) <<"\t"; 
	cout<<intrinsicMat.at<double>(2, 3) <<endl;
	cout<<"==================disCoeffs==========="<<endl;
	cout<<distCoeffs.at<double>(0) <<"\t"; 
	cout<<distCoeffs.at<double>(1) <<"\t"; 
	cout<<distCoeffs.at<double>(2) <<"\t"; 
	cout<<distCoeffs.at<double>(3) <<"\t"; 
	cout<<distCoeffs.at<double>(4) <<endl;

	cout<<"==================intrinsic3x3==========="<<endl;
	cout<<intrinsic.at<double>(0, 0) <<"\t"; 
	cout<<intrinsic.at<double>(0, 1) <<"\t"; 
	cout<<intrinsic.at<double>(0, 2) <<"\n"; 
	cout<<intrinsic.at<double>(1, 0) <<"\t"; 
	cout<<intrinsic.at<double>(1, 1) <<"\t"; 
	cout<<intrinsic.at<double>(1, 2) <<"\n"; 
	cout<<intrinsic.at<double>(2, 0) <<"\t"; 
	cout<<intrinsic.at<double>(2, 1) <<"\t"; 
	cout<<intrinsic.at<double>(2, 2) <<endl; 
}

void getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
    float scale = (max_depth - min_depth)/10;
    if (cur_depth < min_depth) {
        result_r = 0;
        result_g = 0;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale) {
        result_r = 0;
        result_g = int((cur_depth - min_depth) / scale * 255) & 0xff;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale*2) {
        result_r = 0;
        result_g = 0xff;
        result_b = (0xff - int((cur_depth - min_depth - scale) / scale * 255)) & 0xff;
    }
    else if (cur_depth < min_depth + scale*4) {
        result_r = int((cur_depth - min_depth - scale*2) / scale * 255) & 0xff;
        result_g = 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*7) {
        result_r = 0xff;
        result_g = (0xff - int((cur_depth - min_depth - scale*4) / scale * 255)) & 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*10) {
        result_r = 0xff;
        result_g = 0;
        result_b = int((cur_depth - min_depth - scale*7) / scale * 255) & 0xff;
    }
    else {
        result_r = 0xff;
        result_g = 0;
        result_b = 0xff;
    }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class velodyne_lidar
{
public:
	image_transport::Publisher pub;
	ros::Subscriber subCloud;

	velodyne_lidar(ros::NodeHandle nh)
	{
		image_transport::ImageTransport itc(nh);
		subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne/left_front", 2, &velodyne_lidar::pointCloudCallback, this); //接收点云数据，进入回调函数pointCloudCallback
		pub = itc.advertise("/lidar/point_image", 2);
	}
	
																						   										 
					   
private:
	//点云回调函数
	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); //velodyne点云消息包含xyz和intensity
		pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr);										   //把msg消息指针转化为PCL点云
		cv::Mat X(4, 1, cv::DataType<double>::type);
		cv::Mat Y(3, 1, cv::DataType<double>::type);

		int myCount=0;
		cv::Mat src_img(imageMat);
		cv::Mat imageMat_deep = cv::Mat::zeros(H,W,CV_64F);
		for (uint i = 0; i < raw_pcl_ptr->points.size(); i++)
		{
			X.at<double>(0, 0) = raw_pcl_ptr->points[i].x;
			X.at<double>(1, 0) = raw_pcl_ptr->points[i].y;
			X.at<double>(2, 0) = raw_pcl_ptr->points[i].z;
			X.at<double>(3, 0) = 1;
			Y = intrinsicMat * extrinsicMat_RT * X; //雷达坐标转换到相机坐标，相机坐标投影到像素坐标
			cv::Point pt;						   // (x,y) 像素坐标
			pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
			pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);			
			
            
            if (pt.x >= 0 && pt.x < W && pt.y >= 0 && pt.y < H ) //&& raw_pcl_ptr->points[i].x>0去掉图像后方的点云
			{
			
				int u = floor(pt.x + 0.5);
            	int v = floor(pt.y + 0.5);
            	double depth = X.at<double>(2, 0);
            	imageMat_deep.at<double>(v,u)=depth;
          	
        	}


		}
		cv::normalize(imageMat_deep,imageMat_deep,1.0,0.0,cv::NORM_MINMAX);
		cv::Mat output = cv::Mat::zeros(H,W,CV_8U);
		imageMat_deep.convertTo(output,CV_8U,255,0);
		
		sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"mono8",output).toImageMsg();
		pub.publish(msg);
		src_img.release();
  		
	}

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; //

		//去畸变，可选
		// cv::Mat map1, map2;
		// cv::Size imageSize = image.size();		
		// cv::initUndistortRectifyMap(intrinsic, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(intrinsic, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
		// cv::remap(image, image, map1, map2, cv::INTER_LINEAR); // correct the distortion
		//cv::imwrite("1.bmp",image);	
		imageMat = image;
		
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not conveextrinsicMat_RT from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}



int main(int argc, char **argv)

{	
	ros::init(argc, argv, "pointImage_node");
	ros::NodeHandle nh;

	getParameters();
	CalibrationData();
	printMat();

	cout<<"start to work:" <<endl;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/usb_cam0/image_raw", 5, &imageCallback);

	velodyne_lidar llc(nh);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::waitForShutdown();
	return 0;
}
