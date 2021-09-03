#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
 
#include <boost/foreach.hpp>
 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
 
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
 
#include <cv_bridge/cv_bridge.h>
 
// OpenCV2
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
 
 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
using std::cout;
using std::endl;
using std::stringstream;
using std::string;
 
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;
 
unsigned int filesNum = 1;
bool saveCloud = false;
 
void
keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)
{
    if(event.getKeySym() == "space"&& event.keyDown())
        saveCloud = true;
 
}
 
boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback(keyboardEventOccured);
 
    return(v);
}
 
void callback(const ImageConstPtr& image_color_msg, const ImageConstPtr& image_depth_msg)
{
    cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image; // BGR8
    //    cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;
    //    cv_bridge::CvImagePtr image_depth = cv_bridge::toCvCopy(image_depth_msg , sensor_msgs::image_encodings::TYPE_32FC1);
    //    cv::normalize(image_depth->image, image_depth->image, 1, 0, cv::NORM_MINMAX);
 
//    cv_bridge::CvImagePtr image_depth = cv_bridge::toCvCopy(image_depth_msg , sensor_msgs::image_encodings::TYPE_16UC1);
    cv_bridge::CvImagePtr image_depth = cv_bridge::toCvCopy(image_depth_msg);
 
    cv::imshow("color", image_color);
    cv::imshow("depth", image_depth->image);
 
    if(saveCloud)
    {
 
        vector<int>compression_param;
        compression_param.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_param.push_back(100);//  Highest quality
 
        vector<int>d_compression_param;
        d_compression_param.push_back(CV_IMWRITE_PNG_COMPRESSION);
        d_compression_param.push_back(0);// png Highest quality
 
        stringstream stream;
        stream  <<"/home/yake/pcl_gaoxiang/rgbd-slam-tutorial-gx-master/yake_potatochip_kinect/rgb_jpgfile/"<< "rgb"<< filesNum<< ".jpg";
        string filename = stream.str();
 
        stringstream stream2;
        stream2 <<"/home/yake/pcl_gaoxiang/rgbd-slam-tutorial-gx-master/yake_potatochip_kinect/depth_pngfile/"<< "depth"<< filesNum<< ".png";
        string filename2 = stream2.str();
 
 
        imwrite (filename.c_str (), image_color,compression_param);
        //        imwrite (filename2.c_str (), image_depth);
 
        imwrite(filename2.c_str (), image_depth->image, d_compression_param);
 
        cout << filename<<" Saved."<<endl;
        cout << filename2<<" Saved."<<endl;
 
        filesNum++;
        saveCloud = false;
 
    }
 
    cv::waitKey(3);
}
 
 
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");
 
    ros::NodeHandle nh;
 
    cout<< "Press space to save rgb_raw and depth_raw to a file."<<endl;
 
    boost::shared_ptr<visualization::CloudViewer> viewer;
    viewer = createViewer();
 
//    message_filters::Subscriber<Image> image_color_sub(nh,"/camera/rgb/image_raw", 1); // bayer_grbg8
    message_filters::Subscriber<Image> image_color_sub(nh,"/camera/rgb/image_color", 1);// bgr8
 
//     Use the rqt_reconfigure close registration. Otherwise the depth image has no data.(freenect driver)
    message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth/image_raw", 1);// 16UC1
//        message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth/image", 1);// 32FC1
 
//     Open the depth registratioin.
//        message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth_registered/image_raw", 1); // 16UC1
//    message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth_registered/image", 1); // 32FC1
 
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_color_sub, image_depth_sub);
 
    sync.registerCallback(boost::bind(&callback, _1, _2));
 
    ros::Rate rate(30.0);
 
    while (ros::ok() && ! viewer->wasStopped())
    {
        ros::spinOnce();
        rate.sleep();
    }
 
 
    return 0;
}