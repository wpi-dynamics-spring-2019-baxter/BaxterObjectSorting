#include "processPoint.hpp"

processPoint::processPoint(NodeHandle &nh) {
    ROS_INFO_STREAM("inside constructor");
    sub = nh.subscribe < sensor_msgs::PointCloud2 > ("/camera/depth/points", 1, &processPoint::pointCloudCallback, this);
    my_serv = nh.advertiseService("convert2Dto3D", &processPoint::serviceCallback, this);
    ROS_INFO_STREAM("subscribed!!");
}

void processPoint::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    ptcloud_ = cloud;
    //ROS_INFO_STREAM("inside callback");
    // if object detector gives (x,y) then feed the following function (y,x)
    //get3DPoints(248,393);
}

bool processPoint::serviceCallback( baxter_kinect_ros::ServiceFile_2Dto3D::Request &req, baxter_kinect_ros::ServiceFile_2Dto3D::Response &res ){

    Point3f world_points;
    int count = 0;
    int number_of_fruits = req.centroids.size();
    res.d3_points.resize(number_of_fruits);
    for (auto& itr: req.centroids){
        int u = itr.pose.position.x;
        int v = itr.pose.position.y;
        ROS_INFO_STREAM("u v values: "<<u<<","<<v);
        get3DPoints(u,v,world_points);
        ROS_INFO_STREAM("world pts after processing: "<<world_points.x<<" "<<world_points.y<<" "<<world_points.z);
        res.d3_points[count].pose.position.x = world_points.x;
        res.d3_points[count].pose.position.y = world_points.y;
        res.d3_points[count].pose.position.z = world_points.z;
        res.d3_points[count].header.frame_id = itr.header.frame_id;

        ROS_INFO_STREAM("o/p of servcallback: "<< res.d3_points[count].pose.position.x<<" "<<res.d3_points[count].pose.position.y<<" "<<res.d3_points[count].pose.position.z);
        count++;
    }
    return true;
}

void processPoint::get3DPoints(int u, int v, Point3f& world_points) {
    // raw depth value from RGB pixel coordinate
    vector<vector<Point2f>> fruit_centroids;


    int i = (u) + (v)*ptcloud_->width;
    //v = 0 is the first row of the image and u will go from 0 to 640. pt_cloud.width also = 640
    //thus i is like the index of the pixel in the image, pixel at start has i=0, pixel at end of first row
    //has i= 640+0*640 = 640; at end of second row, it is 640 + 1*640 = 1280
    // i for pixel at end = 640 + 480*640


    ROS_INFO_STREAM("inside get3dpoints, width: "<<ptcloud_->height);
// 3D coordinates from point cloud using depth value

//    pcl::PCLPointCloud2 pcl_pc2;
//    pcl_conversions::toPCL(ptcloud_,pcl_pc2);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
//
//    float x3Ddepth = (float)pt_cloud->points[i].x;
//    float y3Ddepth = (float)pt_cloud->points[i].y;
//    float z3Ddepth = (float)pt_cloud->points[i].z;
//    ROS_INFO_STREAM(x3Ddepth<<" "<<y3Ddepth<<" "<<z3Ddepth);

    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(*ptcloud_,output);

    float x = (float)output.points[i].x;
    float y = (float)output.points[i].y;
    float z = (float)output.points[i].z;

    //ROS_INFO_STREAM("printing name"<<name_);

    //Point3f world_points;
    world_points = Point3f(x,y,z);
    ROS_INFO_STREAM("world pts: "<<world_points.x<<" "<<world_points.y<<" "<<world_points.z);

}

