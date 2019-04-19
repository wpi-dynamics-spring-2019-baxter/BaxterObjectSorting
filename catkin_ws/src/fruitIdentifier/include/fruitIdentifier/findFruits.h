//
// Created by akshay on 4/10/19.
//

#ifndef UNTITLED1_FINDFRUITS_H
#define UNTITLED1_FINDFRUITS_H

#endif //UNTITLED1_FINDFRUITS_H
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv/highgui.h"
#include "opencv2/core/core.hpp"
#include "cxcore.h"
#include "highgui.h"
#include "cv.h"
#include <string.h>
#include <experimental/filesystem>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <fruitIdentifier/findCentroid.h>
#include <baxter_kinect_ros/ServiceFile_2Dto3D.h>
//#include <baxter_kinect_ros/processPoint.hpp>

using std::experimental::filesystem::recursive_directory_iterator;
using namespace std;
using namespace cv;
using namespace ros;

class Fruits{
public:
    Fruits(NodeHandle& nh2D);
    void subscriberCallback2D(const sensor_msgs::Image ros_image);
    bool serviceCallback2D(fruitIdentifier::findCentroid::Request& req, fruitIdentifier::findCentroid::Response& res);
    void runAll();
    void createAppleMask();
    void createBananaMask();
    void countPixels();
    void morphOps();
    vector<Moments> findAppleMoments();
    vector<Moments> findBananaMoments();
    vector<Point2f> findAppleCentroids(vector<Moments>);
    vector<Point2f> findBananaCentroids(vector<Moments>);
    void drawAppleContours(vector<Point2f>);
    void drawBananaContours(vector<Point2f>);
    int identifyFruit(vector<Point2f>, vector<Point2f>);

    Mat img;
    Mat hsv_img;
    multimap <int, vector<Point2f> > fruit_centroids;
    int fruit;
    int number_of_fruits;
private:
    double apple_count, banana_count;
    vector<Moments> a_mu, b_mu;
    Mat apple_mask1,apple_mask2,banana_mask1,banana_mask2;
    Mat a1, b1;
    vector<vector<Point>> apple_contours, banana_contours;
    vector<Vec4i> apple_hierarchy, banana_hierarchy;
    Subscriber sub2D;
    ServiceServer my_serv2D;
    ServiceClient client;

};

