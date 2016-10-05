#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <iostream>
#include <fstream>

#include <message_filters/sync_policies/approximate_time.h>

using namespace cv;
using namespace std;
using namespace message_filters;

#define MAX_FRAME 10

#define MAX_X 1200
#define MAX_Z 1200

Mat prevImage;

bool init1 = false;
bool init2 = false;

Mat R_f, t_f; //the final rotation and tranlation vectors

// focal length of the camera
double focal = 2.9;

// principle point of the camera
// TO BE UPDATED
cv::Point2d pp(607.1928, 185.2157);
    
  
/*********************************************************************************************
 * Kanade-Lucas-Tomasi feature tracker is used for finding sparse pixel wise correspondences. 
 * The KLT algorithm assumes that a point in the nearby space, and uses image gradients to 
 * find the best possible motion of the feature point.
 *********************************************************************************************/
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)
{
    // this function track points from points1 in img1 tracks
    // img2 and stores points in point2
    vector<float> err;
    Size winSize=Size(21,21);
    TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
}


void featureDetection(Mat img_1, vector<Point2f>& points1)
{
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    // FAST (Features from Accelerated Segment Test) corner detector
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}  

void rgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb)
{
   ROS_INFO("I heard from rgb");
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg_depth)
{
   ROS_INFO("I heard from depth");
}

// Handler / callback
void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
	ROS_INFO_STREAM("Thread called");
	cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth;

   	try
	{
       	img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s getting depth", e.what());
        return;
    }
    
	try
	{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BAYER_GRBG8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s getting rgb", e.what());
        return;
    }

    Mat& mat_depth = img_ptr_depth->image;
    Mat& mat_rgb = img_ptr_rgb->image;

	Mat currImage;
	cvtColor(mat_rgb, currImage, COLOR_BGR2GRAY);

	// if this is first image store it and go to next iteration
	if(false == init1)	
	{
		prevImage = currImage.clone();
		init1 = true;
		return;
	}	

	Mat E, R, t, mask;
	vector<uchar> status;
	vector<Point2f> currFeatures;
	vector<Point2f> prevFeatures;
        
    featureDetection(prevImage, prevFeatures);
    featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
	
	// RANSAC Random sample consensus
    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        
	if(false == init2)
    {
        R_f = R.clone();
        t_f = t.clone();
        init2 = true;
    }
    else
    {
        t_f = t_f + (R_f*t);
        R_f = R*R_f;
    }
	
	prevImage = currImage.clone();
        
    int myx = int(t_f.at<double>(0));
    int myz = int(t_f.at<double>(2));

	ROS_INFO_STREAM("X = " << myx << " Z = " << myz);
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
  	ros::init(argc, argv, "listner");
  	ros::NodeHandle nh;

	//ros::Subscriber sub1 = nh.subscribe("/camera/rgb/image_raw",1000,rgbCallback);
	//ros::Subscriber sub2 = nh.subscribe("/camera/depth/image",1000,depthCallback);

    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth/image" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_raw" , 1 );


    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

  	// ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();
    
	return 0;
}
