#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <queue>

using namespace cv;
using namespace std;
using namespace message_filters;

#define MAX_X 1200
#define MAX_Z 1200

Mat prevImageRGB;
Mat prevImageDepth;
Mat R_f, t_f; //the final rotation and tranlation vectors

// focal length of the camera
double focal = 525;

int cnt = 0;

// principle point of the camera
cv::Point2d pp(312.60,228.62);

enum States
{
    boot = 0,
    init,
    processing
};

States state = boot;

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

    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
    {  
        if (status.at(i) == 0)	
        {
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}
    }
}

void featureDetection(Mat img_1, vector<Point2f>& points1)
{
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    //FAST (Features from Accelerated Segment Test) corner detector
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
}  

// Handler / callback
void callback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth)
{
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

    Rect myROI(0,105,590,375);

    Mat& imageDepth = img_ptr_depth->image;
    Mat& imageRGB = img_ptr_rgb->image;

    Mat currImageRGB_f;
    Mat currImageRGB_cf1;

    cvtColor( imageRGB, currImageRGB_cf1, COLOR_BayerGB2BGR_EA);
    
    Mat currImageRGB_cf;
    bilateralFilter( currImageRGB_cf1, currImageRGB_cf, 9, 75, 75);
   
    cvtColor( currImageRGB_cf, currImageRGB_f, COLOR_BGR2GRAY);
      
    Mat currImageDepth; // (imageDepth,myROI);
    Mat currImageRGB = currImageRGB_f; //(currImageRGB_f,myROI);
    Mat currImageRGB_c = currImageRGB_cf; //(currImageRGB_cf,myROI);
    
    imageDepth.convertTo(imageDepth, CV_8U, 0.1);

    //cvtColor( currImageRGB_bayer, currImageRGB, COLOR_BayerGB2GRAY);

	//GaussianBlur( currImageRGB, currImageRGB, Size(5,5), 0, 0);	

    //vector<int> png_parameters;
    //png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION);

    // if this is first image store it and go to next iteration
    if(state == boot)    
    {
        prevImageRGB = currImageRGB.clone();
        prevImageDepth = currImageDepth.clone();
        state = init;
        return;
    }    

    Mat E, R, t, mask;
    vector<uchar> status;
    vector<Point2f> currFeatures;
    vector<Point2f> prevFeatures;
        
    featureDetection(prevImageRGB, prevFeatures);

    featureTracking(prevImageRGB, currImageRGB, prevFeatures, currFeatures, status);
   
    // RANSAC Random sample consensus
    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    int count = 0;     
    // calculate centroid for prev and current image
    double sumXPrev = 0, sumYPrev = 0, sumZPrev = 0;
    double sumXCurr = 0, sumYCurr = 0, sumZCurr = 0;  

    for(int i=0;i<mask.rows;i++)
    {
        // mask 1 means inliers
        if(1 ==  mask.at<uchar>(i,1) 
            && prevImageDepth.at<unsigned short>(prevFeatures.at(i)) != 0
            && currImageDepth.at<unsigned short>(currFeatures.at(i)) != 0)
        {
            //sumXPrev +=                
            sumZPrev += prevImageDepth.at<unsigned short>(prevFeatures.at(i));
            sumZCurr += currImageDepth.at<unsigned short>(currFeatures.at(i));
            count++;
        }
    }     

    cnt++;

    //Mat zeros = Mat::zeros(currImageRGB.rows,currImageRGB.cols,CV_8UC3);    

    if(state == init)
    {
        R_f = R.clone();
        t_f = t.clone();
        state = processing;
    }
    else
    {
        t_f = t_f + scale * (R_f*t);
        R_f = R*R_f;
    }
           
    prevImageRGB = currImageRGB.clone();
    prevImageDepth = currImageDepth.clone();
    
    ROS_INFO_STREAM(mask.rows);

    ROS_INFO_STREAM(currFeatures.size());

    for(int i=0;i<mask.rows;i++)
    {
        if(1 ==  mask.at<uchar>(i,1))
        {
            circle(currImageRGB_c,currFeatures.at(i),2,CV_RGB(255,0,0));
            line(currImageRGB_c,prevFeatures.at(i),currFeatures.at(i),CV_RGB(0,255,0));
        }
    }

    //imshow("imagergb",currImageRGB-prevImageRGB);
    imshow("Blur", currImageDepth);    
    imshow("imager",currImageRGB_c); 
    waitKey(1);
    
    int myx = int(t_f.at<double>(0));
    int myz = int(t_f.at<double>(2));
    
    Mat mr,mq;
    Vec3d angles = RQDecomp3x3(R_f,mr,mq);

    ROS_INFO_STREAM(t_f);
    //ROS_INFO_STREAM("*X = " << myx << " Y = " << myz << " " << scale);
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "optical_flow_node");
    ros::NodeHandle nh;
   
    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/image_raw" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "camera/rgb/image_raw" , 1 );

    ///mobile_base/commands/velocity

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    // ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();    
       
    return 0;
}
