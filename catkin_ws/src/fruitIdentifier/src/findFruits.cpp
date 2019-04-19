#include "findFruits.h"

Fruits::Fruits(NodeHandle& nh2D){
    ROS_INFO_STREAM("inside constructor");
    sub2D = nh2D.subscribe < sensor_msgs::Image > ("/camera/depth/image_raw", 1, &Fruits::subscriberCallback2D, this);
    my_serv2D = nh2D.advertiseService("get_fruits", &Fruits::serviceCallback2D, this);
    client = nh2D.serviceClient<baxter_kinect_ros::ServiceFile_2Dto3D>("/convert2Dto3D");

}

void Fruits:: subscriberCallback2D(const sensor_msgs::Image ros_image) {
    ROS_INFO_STREAM("inside subscriber2D");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    img = cv_ptr->image;
    this->runAll();
}

bool Fruits:: serviceCallback2D(fruitIdentifier::findCentroid::Request& req, fruitIdentifier::findCentroid::Response& res){
    //res.d3_points[0].header.frame_id = fruit;
    ROS_INFO_STREAM("inside this service");
    for(auto& mapp: fruit_centroids)
        cout<<mapp.first<<": "<<mapp.second<<endl;
    cout<<"mapp: "<<fruit_centroids.count(2);

    geometry_msgs::PoseStamped centroid_values [number_of_fruits];
    ROS_INFO_STREAM("number"<<number_of_fruits);

    res.final_3d_points.resize(number_of_fruits);
    int count = 0;
    baxter_kinect_ros::ServiceFile_2Dto3D srv;
    srv.request.centroids.resize(number_of_fruits);
    ROS_INFO_STREAM("filling request");
    for (auto& itr: fruit_centroids){
        for(auto& itr2: itr.second) {
            centroid_values[count].header.frame_id = itr.first;
            centroid_values[count].pose.position.x = itr2.x;
            centroid_values[count].pose.position.y = itr2.y;
            srv.request.centroids[count] = centroid_values[count];
            count++;
        }
    }


    if(client.call(srv)){
        ROS_INFO_STREAM("Successfully called the service to convert the 2D centroids to 3D points");
        //string frameID = srv.response.d3_points[0].header.frame_id;
        res.final_3d_points = srv.response.d3_points;
        ROS_INFO_STREAM("Successfully called the service to convert the 2D centroids to 3D points");
        //ROS_INFO_STREAM("Fruit:   "<<frameID.c_str());

        ROS_INFO_STREAM("x coord: "<<res.final_3d_points[0].pose.position.x);
        ROS_INFO_STREAM("y coord: "<<res.final_3d_points[0].pose.position.y);
        ROS_INFO_STREAM("z coord: "<<res.final_3d_points[0].pose.position.z);

    }
    else{
        ROS_ERROR("unable to call serv to convert 2D to 3D");
        return 1;
    }
    return true;
}

void Fruits::runAll(){
    fruit_centroids.clear();
    cvtColor(this->img, this->hsv_img, COLOR_BGR2HSV);
    //this->setImages(img, &hsv_img);

    this->createAppleMask();
    this->createBananaMask();

    this->morphOps();
    this->countPixels();

    vector<Moments> a_mu = this->findAppleMoments();
    vector<Moments> b_mu = this->findBananaMoments();

    vector<Point2f> apple_centroids  = this->findAppleCentroids(a_mu);
    vector<Point2f> banana_centroids = this->findBananaCentroids(b_mu);
    vector<Point2f> final_banana_centroids;
    vector<Point2f> final_apple_centroids;

    this->drawAppleContours(apple_centroids);
    this->drawBananaContours(banana_centroids);

    this->fruit = this->identifyFruit(apple_centroids, banana_centroids);
    vector<Point2f> temp;
    switch (fruit){
        case 0:
            cout<<"neither";
            temp.emplace_back(Point2f(0,0));
            fruit_centroids.emplace(make_pair(fruit, temp));
            break;
        case 1:
            cout<<"its an apple\n";
            for (auto& itr: apple_centroids) {
                if (itr.x && itr.y) {

                    final_apple_centroids.emplace_back(Point2f(itr.x, itr.y));
                    cout << itr.x << "," << itr.y << endl;
                }
            }
            this->fruit_centroids.emplace(make_pair(fruit, final_apple_centroids));

            break;
        case 2:
            cout<<"its a banana\n";
            for (auto& itr: banana_centroids) {
                if (itr.x && itr.y) {

                    final_banana_centroids.emplace_back(Point2f(itr.x, itr.y));
                    cout << itr.x << "," << itr.y << endl;
                }
            }
            this->fruit_centroids.emplace(make_pair(fruit, final_banana_centroids));

            break;
        case 3:
            cout<<"its both!\n";
            cout<<"apple centroids:\n";
            for (auto& itr: apple_centroids) {
                if (itr.x && itr.y) {

                    final_apple_centroids.emplace_back(Point2f(itr.x, itr.y));
                    cout << itr.x << "," << itr.y << endl;
                }
            }
            this->fruit_centroids.emplace(make_pair(1, final_apple_centroids));
            cout<<"banana centroids:\n";
            for (auto& itr: banana_centroids) {
                if (itr.x && itr.y) {
                    final_banana_centroids.emplace_back(Point2f(itr.x, itr.y));

                    cout << itr.x << "," << itr.y << endl;
                }
            }
            this->fruit_centroids.emplace(make_pair(2, final_banana_centroids));

            break;
    }
    number_of_fruits = final_apple_centroids.size() + final_banana_centroids.size();
    cout<<"final size: "<<final_apple_centroids.size() + final_banana_centroids.size()<<endl;
}

void Fruits::createAppleMask(){
    inRange(hsv_img, Scalar(0,0,220), Scalar(10,255,241), apple_mask1);
    inRange(hsv_img, Scalar(0,0,35), Scalar(10,255,140), apple_mask2);
    apple_mask1 = apple_mask1+apple_mask2;
    cvNamedWindow("color", CV_WINDOW_NORMAL);

    imshow("color", img);

    waitKey(1000);
}

void Fruits::createBananaMask(){
    inRange(hsv_img, Scalar(30,150,30), Scalar(50,255,70), banana_mask1);
    inRange(hsv_img, Scalar(0,0,0), Scalar(1,1,1), banana_mask2);
    banana_mask1 = banana_mask1+banana_mask2;
    cvNamedWindow("color", CV_WINDOW_NORMAL);

    imshow("color", img);

}

void Fruits::countPixels(){
    apple_count = 0;
    banana_count = 0;
    for(int i =0; i<a1.rows; i++){
        for(int j=0; j<a1.cols; j++){
            if(a1.at<Vec3b>(i,j)[0] == 255)
                apple_count++;
        }
    }

    for(int i =0; i<b1.rows; i++){
        for(int j=0; j<b1.cols; j++){
            if(b1.at<Vec3b>(i,j)[0] == 255)
                banana_count++;
        }
    }


}

// 0 - neither
// 1 - apple
// 2 - banana
// 3 - both
int Fruits::identifyFruit(vector<Point2f> apple_centroids, vector<Point2f> banana_centroids) {
    cout<<"debug identifyFruit\n";
    cout<<"apple_count"<<apple_count<<" applecent size: "<<apple_centroids.size()<<" banana_count: "<<banana_count<<" ban cent size: "<<banana_centroids.size()<<endl;
    //if(apple_count>10 && apple_centroids.size() && banana_count<10)
    if((apple_count>10 && apple_centroids.size()) && (banana_count>10 && banana_centroids.size()))
        return 3;
    else if(apple_count>10 && apple_centroids.size())
        return 1;
    //if(apple_count<10 && banana_count>10 && banana_centroids.size())
    else if(banana_count>10 && banana_centroids.size())
        return 2;
    else
        return 0;
}
void Fruits::morphOps(){
    dilate(apple_mask1, a1, Mat(),Point(-1,-1),3);
    erode(a1, a1, Mat(),Point(-1,-1),6);
    dilate(a1, a1, Mat(),Point(-1,-1),3);
    //erode(a1, a1, Mat(),Point(-1,-1),6);
    dilate(a1, a1, Mat(),Point(-1,-1),3);


    dilate(banana_mask1, b1, Mat(),Point(-1,-1),3);
    erode(b1, b1, Mat(),Point(-1,-1),6);
    dilate(b1, b1, Mat(),Point(-1,-1),3);

    dilate(b1, b1, Mat(),Point(-1,-1),3);

}

vector<Moments> Fruits:: findAppleMoments(){


    findContours( a1, apple_contours, apple_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


    // get the moments of apples
    vector<Moments> a_mu(apple_contours.size());
    for( int i = 0; i<apple_contours.size(); i++ )
    { a_mu[i] = moments( apple_contours[i], false ); }
    for(int i =0; i<a_mu.size(); i++)
        return a_mu;
}

vector<Moments> Fruits:: findBananaMoments(){


    findContours( b1, banana_contours, banana_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );


    // get the moments of bananas
    vector<Moments> b_mu(banana_contours.size());
    for( int i = 0; i<banana_contours.size(); i++ )
    { b_mu[i] = moments( banana_contours[i], false ); }

    return b_mu;

}

vector<Point2f> Fruits:: findAppleCentroids(vector<Moments> a_mu){


    vector<Point2f> apple_centroids(apple_contours.size());

    for( int i = 0; i<apple_contours.size(); i++)
    {
        double apple_arclength = arcLength(Mat(apple_contours[i]),true);
        cout<<"arc length: "<<apple_arclength<<endl;
        if(a_mu[i].m00!=0 && apple_arclength>100){

            apple_centroids[i] = Point2f( a_mu[i].m10/a_mu[i].m00 , a_mu[i].m01/a_mu[i].m00 );
            cout<<"apple centroids: "<<apple_centroids[i].x<<","<<apple_centroids[i].y<<endl;
        }}

    return apple_centroids;
}

vector<Point2f> Fruits:: findBananaCentroids(vector<Moments> b_mu){
    // get the centroid of bananas.
    vector<Point2f> banana_centroids(banana_contours.size());

    for( int i = 0; i<banana_contours.size(); i++){
        double banana_arclength = arcLength(Mat(banana_contours[i]),true);
        if(b_mu[i].m00!=0 && banana_arclength>200){

            banana_centroids[i] = Point2f( b_mu[i].m10/b_mu[i].m00 , b_mu[i].m01/b_mu[i].m00 );
            //cout<<"banana centroids: "<<banana_centroids[i].x<<","<<banana_centroids[i].y<<endl;
        }}

    return banana_centroids;
}

void Fruits :: drawAppleContours(vector<Point2f> apple_centroids){
    // draw contours
    Mat drawing(a1.size(), CV_8UC3, Scalar(255,255,255));
    for( int i = 0; i<apple_contours.size(); i++ )
    {
        Scalar color = Scalar(167,151,0); // B G R values
        Scalar color1 = Scalar(0,0,255); // B G R values
        drawContours(drawing, apple_contours, i, color, 2, 8, apple_hierarchy, 0, Point());
        //dilate(drawing, drawing, Mat(),Point(-1,-1),3);
        //erode(drawing, drawing, Mat(),Point(-1,-1),6);
        //dilate(drawing, drawing, Mat(),Point(-1,-1),3);
        circle( drawing, apple_centroids[i], 8, color1, -1, 8, 0 );
    }
    namedWindow( "Apple Contours", CV_WINDOW_NORMAL );
    imshow( "Apple Contours", drawing );
    waitKey(1000);
}
void Fruits :: drawBananaContours(vector<Point2f> banana_centroids){
    Mat drawing1(b1.size(), CV_8UC3, Scalar(255,255,255));
    for( int i = 0; i<banana_contours.size(); i++ )
    {
        Scalar color = Scalar(167,151,0); // B G R values
        Scalar color1 = Scalar(0,0,255); // B G R values
        drawContours(drawing1, banana_contours, i, color, 2, 8, banana_hierarchy, 0, Point());
        circle( drawing1, banana_centroids[i], 8, color1, -1, 8, 0 );
    }
    namedWindow( "Banana Contours", CV_WINDOW_NORMAL );
    imshow( "Banana Contours", drawing1 );
    waitKey(1000);
}//
// Created by akshay on 4/10/19.
//

