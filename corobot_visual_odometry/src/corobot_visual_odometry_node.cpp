#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <bitset>
#include <algorithm>

#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/rift.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>

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

using namespace std;
using namespace cv;
using namespace pcl;

// Types
typedef PointXYZRGB PointIn;
typedef PointCloud<PointIn> PointCloudIn;

typedef PointXYZI KeyPointT;
typedef PointCloud<KeyPointT> KeyPointCloudT;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;


enum States
{
    boot = 0,
    init,
    processing
};

double fx = 500.9883833686623;
double fy = 501.7202166988847;

double cx = 317.8102802776187;
double cy = 248.4374439271775;

double inlierThresh = 9;

States state = boot;

PointCloudIn::Ptr prevC;

Mat prevImageRGB;
Mat prevImageDepth;

//Mat R_f,t_f;

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
    //FAST (Features from Accelerated Segment Test) corner detector
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, points1, vector<int>());
} 

double squaredDistanceBetween(KeyPointT point1, KeyPointT point2)
{
    return (point1.x - point2.x) * (point1.x - point2.x) +
            (point1.y - point2.y) * (point1.y - point2.y) +
            (point1.z - point2.z) * (point1.z - point2.z);
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

    ROS_INFO("Received images");

    Mat& imageDepth = img_ptr_depth->image;
    Mat& imageRGB = img_ptr_rgb->image;

    Mat currImageRGB_f;
    Mat currImageRGB_cf1;

    cvtColor( imageRGB, currImageRGB_cf1, COLOR_BayerGB2BGR_EA);
    
    Mat currImageRGB_cf;
    bilateralFilter( currImageRGB_cf1, currImageRGB_cf, 9, 75, 75);

    PointCloudIn::Ptr curr (new PointCloudIn);
    curr->header.frame_id = "camera_depth_frame";
    curr->height = currImageRGB_cf.rows;
    curr->width = currImageRGB_cf.cols;

    curr->resize(curr->height * curr->width);

    ROS_INFO("Creating point cloud");

    int i = 0;

    for(int y = 0; y < currImageRGB_cf.rows; y++)
    {
        for(int x = 0; x < currImageRGB_cf.cols; x++)
        {
            PointIn& temp = curr->points[i];
            i++;
            temp.z = imageDepth.at<unsigned short>(y,x)/10;            
            temp.x = (x - cx) * temp.z / fx; 
            temp.y = (y - cy) * temp.z / fy;
            temp.b = currImageRGB_cf.at<cv::Vec3b>(y,x)[0];
            temp.g = currImageRGB_cf.at<cv::Vec3b>(y,x)[1];
            temp.r = currImageRGB_cf.at<cv::Vec3b>(y,x)[2];
        }         
    }    

    curr->sensor_origin_.setZero ();
    curr->sensor_orientation_.w () = 0.0f;
    curr->sensor_orientation_.x () = 0.0f;
    curr->sensor_orientation_.y () = 0.0f;
    curr->sensor_orientation_.z () = 0.0f;   

    cvtColor( currImageRGB_cf, currImageRGB_f, COLOR_BGR2GRAY);
      
    Mat currImageDepth = imageDepth; // (imageDepth,myROI);
    Mat currImageRGB = currImageRGB_f; //(currImageRGB_f,myROI);
    Mat currImageRGB_c = currImageRGB_cf; //(currImageRGB_cf,myROI);

    // if this is first image store it and go to next iteration
    if(state == boot)    
    {
        prevC = curr;
        prevImageRGB = currImageRGB.clone();
        prevImageDepth = currImageDepth.clone();
        state = init;
        return;
    }    

    vector<uchar> status;
    vector<Point2f> currFeatures;
    vector<Point2f> prevFeatures;
        
    featureDetection(prevImageRGB, prevFeatures);
    featureTracking(prevImageRGB, currImageRGB, prevFeatures, currFeatures, status);   

    KeyPointCloudT::Ptr prevKeypoints (new KeyPointCloudT);
    KeyPointCloudT::Ptr currKeypoints (new KeyPointCloudT);

    for(int i=0;i<status.size();i++)
    {
        // mask 1 means inliers
        if(status.at(i) == 1 
            && prevImageDepth.at<unsigned short>(prevFeatures.at(i)) != 0
            && currImageDepth.at<unsigned short>(currFeatures.at(i)) != 0)
        {
            // create 3D points
            KeyPointT prevPoint;
            prevPoint.z = prevImageDepth.at<unsigned short>(prevFeatures.at(i))/10;
            prevPoint.x = (prevFeatures.at(i).x - cx) * prevPoint.z / fx; 
            prevPoint.y = (prevFeatures.at(i).y - cy) * prevPoint.z / fy;            
            prevPoint.intensity = prevImageRGB.at<unsigned short>(prevFeatures.at(i));
            prevKeypoints->push_back(prevPoint);
            
            KeyPointT currPoint;  
            currPoint.z = currImageDepth.at<unsigned short>(currFeatures.at(i))/10;           
            currPoint.x = (currFeatures.at(i).x - cx) * currPoint.z / fx; 
            currPoint.y = (currFeatures.at(i).y - cy) * currPoint.z / fy;    
            currPoint.intensity = currImageRGB.at<unsigned short>(currFeatures.at(i));
            currKeypoints->push_back(currPoint);        

            circle(currImageRGB_c,currFeatures.at(i),2,CV_RGB(255,0,0));
            circle(currImageRGB_c,prevFeatures.at(i),2,CV_RGB(0,255,0));
            line(currImageRGB_c,prevFeatures.at(i),currFeatures.at(i),CV_RGB(0,0,255));
        }
    }   

    int size = prevKeypoints->size();

    // inlier detection for keypoints
    bool m[size][size];    

    // create adjuncey matrix
    for(int i = 0; i < size; i++)
    {
        for(int j = i; j < size; j++)
        {
            if(abs (squaredDistanceBetween(currKeypoints->at(i),currKeypoints->at(j))
                - squaredDistanceBetween(prevKeypoints->at(i),prevKeypoints->at(j))) 
                < inlierThresh)
            {
                m[i][j] = true;
                m[j][i] = true;
            }
            else
            {
                m[i][j] = false;
                m[j][i] = false;  
            }
        }
    }

    vector<int> clique;
    vector<bool> potentialNodes(size,true);
    bool anyPotentialNode = true;

    while(anyPotentialNode)
    {
        // find node with max edges
        int edgeMax = 0;
        int currMax = 0;

        for(int i = 0; i < size; i++)
        {
            if(potentialNodes[i])
            {
                int edgeCount = 0;
                for(int j = 0; j < size; j++)
                {
                    if(m[i][j] && potentialNodes[j])
                    {
                        edgeCount++;
                    }
                }
                if(edgeCount >= edgeMax)
                {
                    edgeMax = edgeCount;
                    currMax = i;
                }
            }   
        }

        if(edgeMax > 0)
        {
            clique.push_back(currMax);
        }
        else
        {
            // important***************************************************************************
            ROS_INFO_STREAM("No inliers");
            break;
        }
    
        // intialize potentialNodes
        for(int i = 0; i < size; i++)
        {
            potentialNodes[i] = m[i][clique[0]];
        }

        // only keep nodes which are connected to all nodes in clique
        for(int i = 1; i < clique.size(); i++)
        {
            for(int j = 0; j < size; j++)
            {
                potentialNodes[j] = potentialNodes[j] & m[j][clique[i]];
            }
        }

        // neglect nodes which we already have
        for(int i = 1; i < clique.size(); i++)
        {
            potentialNodes[clique[i]] = false;
        }

        anyPotentialNode = false;
        for(int i = 0; i < size; i++)
        {
            if(potentialNodes[i])
            {
                anyPotentialNode = true;
                break;    
            }
        } 
    }

    // remove keypoints which are not in clique
    ROS_INFO_STREAM(clique.size());
    ROS_INFO_STREAM("****************************************************************");

    CorrespondencesPtr intial_correspondences(new pcl::Correspondences);
    intial_correspondences->resize(clique.size());
    for(int i =0; i < clique.size(); i++)
    {
        (*intial_correspondences)[i].index_query = clique[i];
        (*intial_correspondences)[i].index_match = clique[i];
    }    

    /*prevKeypoints->sensor_origin_.setZero ();
    prevKeypoints->sensor_orientation_.w () = 0.0f;
    prevKeypoints->sensor_orientation_.x () = 1.0f;
    prevKeypoints->sensor_orientation_.y () = 0.0f;
    prevKeypoints->sensor_orientation_.z () = 0.0f;

    currKeypoints->sensor_origin_.setZero ();
    currKeypoints->sensor_orientation_.w () = 0.0f;
    currKeypoints->sensor_orientation_.x () = 1.0f;
    currKeypoints->sensor_orientation_.y () = 0.0f;
    currKeypoints->sensor_orientation_.z () = 0.0f;*/  

    ROS_INFO_STREAM("Finding final correspondance.");

    CorrespondencesPtr final_correspondences(new pcl::Correspondences);
    registration::CorrespondenceRejectorSampleConsensus<KeyPointT> rejector;
    rejector.setInputSource (prevKeypoints);
    rejector.setInputTarget (currKeypoints);
    rejector.setInputCorrespondences(intial_correspondences);
    rejector.getCorrespondences(*final_correspondences);    
    Eigen::Matrix4f transformation = rejector.getBestTransformation();
    
    ROS_INFO_STREAM(intial_correspondences->size());
    ROS_INFO_STREAM(final_correspondences->size());

    ROS_INFO_STREAM("Finding Transformation matrix.");

    ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    ROS_INFO("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    ROS_INFO("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));            

    prevImageRGB = currImageRGB.clone();
    prevImageDepth = currImageDepth.clone();

    imshow("imager",currImageRGB_c); 
    waitKey(1);

    /*pcl::visualization::PCLVisualizer vis;
    //add the first cloud to the viewer
    vis.addPointCloud (prevC->makeShared(), "src_points");
       
    //transfor the second cloud to be able to view them without overlaying each other
    Eigen::Matrix4f t;
    t<<1,0,0,500,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;
    
    ROS_INFO("Transforming cloud 1");
    pcl::PointCloud<pcl::PointXYZRGB> cloudview;
    pcl::transformPointCloud(*curr,cloudview,t);

    //add the second point cloud
    vis.addPointCloud (cloudview.makeShared(), "tgt_points");

    bool alter=false;
               
    //copy the cloudNext Keypoints to keypointsDisplay to prevent altering the data in p_tgt.x+=50;
    ROS_INFO("Adding cloud 2");    
    pcl::PointCloud<KeyPointT> keypointDisplay;
    pcl::copyPointCloud<KeyPointT> (*currKeypoints,keypointDisplay);
    ROS_INFO("Adding correspondance");

    KeyPointT tempo;
    tempo.x = 0;
    tempo.y = 0;
    tempo.z = 0;
    tempo.intensity = 0;
   
    //vis.addSphere(tempo,500,255,0,0,"Center");


    KeyPointT tempo2;
    tempo2.x = 0;
    tempo2.y = 0;
    tempo2.z = 10000;
    tempo2.intensity = 0;
   
    //vis.addSphere(tempo2,500,255,0,0,"Center2");

    for (size_t i =0; i <final_correspondences->size (); i++)
    { 
        KeyPointT & p_src = (*prevKeypoints).points.at((*final_correspondences)[i].index_query);
        KeyPointT & p_tgt = keypointDisplay.points.at((*final_correspondences)[i].index_match);
     
        p_tgt.x+=500;

        // Draw the line
        std::stringstream ss ("line");
        ss << i;
        std::stringstream sss ("spheresource");
        sss << i;
        std::stringstream ssss ("spheretarget");
        ssss << i;
        if(alter)
        {
            vis.addSphere(p_src,0.05,255,0,0,sss.str());
            vis.addSphere(p_tgt,0.05,255,0,0,ssss.str());
            vis.addLine (p_src, p_tgt, 0, 0, 255, ss.str ());       
        }
        else
        {
            vis.addSphere(p_src,0.05,255,255,0,sss.str());
            vis.addSphere(p_tgt,0.05,255,255,0,ssss.str());
            vis.addLine (p_src, p_tgt, 220, 24, 225, ss.str ());
        }
        alter != alter;
    }
    ROS_INFO("Displaying clouds");
    vis.resetCamera ();
    vis.spin (); 
    ROS_INFO("Complete");*/
    prevC = curr;

}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "visual_odometry_node");
    ros::NodeHandle nh;
   
    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/image_raw" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "camera/rgb/image_raw" , 1 );

    ///mobile_base/commands/velocity

    // ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();    
       
    return 0;
}
