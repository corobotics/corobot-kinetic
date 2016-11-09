#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>

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
double focal = 2.9;

int cnt = 0;

// principle point of the camera
// TO BE UPDATED
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
    calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 2, termcrit, 0, 0.001);
}

void featureDetection(Mat img_1, vector<Point2f>& points1)
{
    vector<KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    //FAST (Features from Accelerated Segment Test) corner detector
    //FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	cv::Ptr<Feature2D> detector = cv::xfeatures2d::SIFT::create();
	detector->detect(img_1,keypoints_1);
    KeyPoint::convert(keypoints_1, points1, vector<int>());

    //goodFeaturesToTrack(img_1, points1, 20, 0.05, 5.0, cv::Mat());    
}  

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// Handler / callback
void callback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth)
{
    cnt++;
    if(cnt%30 != 0)
    {
    //    return;    
    }
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

    //Rect myROI(0,105,590,375);

    Mat& currImageDepth = img_ptr_depth->image;
    Mat& currImageRGB = img_ptr_rgb->image;

    //Mat currImageDepth(imageDepth,myROI);
    //Mat currImageRGB(imageRGB,myROI);

	GaussianBlur( currImageDepth, currImageDepth, Size(9,9), 0, 0);	
	
    vector<int> png_parameters;
    png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION);

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
    int tempo = recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    // calculate centroid for prev and current image
    double sumXPrev = 0, sumYPrev = 0, sumZPrev = 0;
    double sumXCurr = 0, sumYCurr = 0, sumZCurr = 0;  
    int count = 0;  

    for(int i=0;i<mask.rows;i++)
    {
        // mask 1 means inliers
        if(1 ==  mask.at<uchar>(i,1))
        {
            // convert from mm to cm
            sumZPrev += prevImageDepth.at<unsigned short>(prevFeatures.at(i)) / 100;
            sumZCurr += currImageDepth.at<unsigned short>(currFeatures.at(i)) / 100;
            count++;
        }
    }     
    
    double scale = 1;//(sumZPrev - sumZCurr) / count;
    
    // Currently threshold is set to 0    
    if(scale < 0)
    {
        scale =0;
    }

    Mat blur_img,blur_imgc;
    double minVal, maxVal;
    minMaxLoc(img_ptr_depth->image, &minVal, &maxVal); //find minimum and maximum intensities
    //Mat draw;
    //img_ptr_depth->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    //currImageDepth.convertTo(blur_imgc, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    //imwrite("odimage" + to_string(cnt) + ".PNG",blur_img,png_parameters);  
    //imwrite("dimage" + to_string(cnt) + ".PNG",blur_imgc,png_parameters);
    //imwrite("orimage" + to_string(cnt) + ".PNG",imageRGB,png_parameters);  
    //imwrite("rimage" + to_string(cnt) + ".PNG",currImageRGB,png_parameters);
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
        
    for(int i=0;i<mask.rows;i++)
    {
        if(1 ==  mask.at<uchar>(i,1))
        {
            line(currImageRGB,prevFeatures.at(i),currFeatures.at(i),CV_RGB(255,0,0));
        }
    }

    imshow("imagergb",currImageRGB);
    //imshow("imagedepth",currImageDepth);
    //imshow("Blur", blur_img);    
    waitKey(1);
    
    int myx = int(t_f.at<double>(0));
    int myz = int(t_f.at<double>(2));
    
    Mat mr,mq;
    Vec3d angles = RQDecomp3x3(R,mr,mq);

    ROS_INFO_STREAM(angles.t());
    //ROS_INFO_STREAM("*X = " << myx << " Y = " << myz << " " << scale);
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "optical_flow_node");
    ros::NodeHandle nh;
   
    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "camera/depth/image_raw" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "camera/rgb/image_raw" , 1 );


    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    // ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();    
       
    return 0;
}
