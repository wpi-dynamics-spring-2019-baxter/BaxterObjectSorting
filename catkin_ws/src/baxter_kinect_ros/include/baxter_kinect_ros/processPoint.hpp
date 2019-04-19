#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <baxter_kinect_ros/ServiceFile_2Dto3D.h>
//#include "opencv/highgui.h"
//#include "opencv2/highgui/highgui.hpp"
//#include "cxcore.h"
//#include "highgui.h"
//#include "cv.h"

using namespace ros;
using namespace sensor_msgs;
using namespace std;
using namespace cv;

class processPoint{
    Subscriber sub;
    ServiceServer my_serv;
    PointCloud2ConstPtr ptcloud_;

public:
    processPoint(NodeHandle& nh);

    ~processPoint(){

    }
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    bool serviceCallback( baxter_kinect_ros::ServiceFile_2Dto3D::Request &req, baxter_kinect_ros::ServiceFile_2Dto3D::Response &res );
    void get3DPoints(int u,int v, Point3f& world_points);
};