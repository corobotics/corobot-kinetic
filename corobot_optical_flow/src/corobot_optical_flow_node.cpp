#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/brisk_2d.h>
#include <pcl/keypoints/brisk_2d.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace pcl;

// Types
typedef PointCloud<PointXYZRGB> PointCloudIn;

typedef PointXYZI PointT;
typedef PointCloud<PointT> PointCloudT;

//typedef PointCloud<Normal> PointCloudN;

typedef PointWithScale KeyPointT;
typedef PointCloud<KeyPointT> KeyPointCloudT;

typedef BRISKSignature512 FeatureT;
typedef PointCloud<FeatureT> FeatureCloudT;

enum States
{
    boot = 0,
    init,
    processing
};

States state = boot;

PointCloudT::Ptr prev;
FeatureCloudT::Ptr prev_features;  

Mat R_f,t_f;

// Handler / callback
void callback(const sensor_msgs::PointCloud2ConstPtr& pointer)
{
    PointCloudIn::Ptr input (new PointCloudT);

    PointCloudT::Ptr curr (new PointCloudT);
    //PointCloudN::Ptr currN(new PointCloudNT);

    KeyPointCloudT::Ptr curr_KeyPoints (new KeyPointCloudT);

	FeatureCloudT::Ptr curr_features (new FeatureCloudT);
    
    ROS_INFO_STREAM("Called0");
    
    fromROSMsg(*pointer, *input);
    PointXYZRGBtoXYZI(*input,*currT)

    ROS_INFO_STREAM("Called1");

    // Apply filter (noise removal) on the scene
    

    // Estimate normals for scene
    NormalEstimation<PointXYZI,PointN> nest;
    nest.setSearchMethod (search::KdTree<PointXYZI>::Ptr (new search::KdTree<PointXYZI> (false)));
    nest.setRadiusSearch (0.03);
    nest.setInputCloud (currT);
    nest.compute (*currN);
    
    ROS_INFO_STREAM("1:" << currT->size ());
    ROS_INFO_STREAM("2:" << currN->size ());

    ROS_INFO_STREAM("Called2");

    // Find  keypoints    
  
    ROS_INFO_STREAM("Called3");
    BriskKeypoint2d<PointT> bkest;
    bkest.setThreshold(60);
    bkest.setOctaves(4);
    bkest.setInputCloud(currT);
    bkest.compute(*curr_KeyPoints);

    // Estimate features for krypoints 
    BRISK2DEstimation<PointT> best;
    best.setInputCloud (currT);
    best.setKeyPoints (curr_KeyPoints);
    best.compute (*curr_features);

    ROS_INFO_STREAM("Called4");
    ROS_INFO_STREAM(":" << curr_features->size ());
    
    // if this is first image store it and go to next iteration
    if(state == boot)    
    {
        prev = curr;
        prev_features = curr_features;
        state = init;        
        return;
    }    
        
    /*pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
    align.setInputSource (prevT);
    align.setSourceFeatures (prev_features);
    align.setInputTarget (currT);
    align.setTargetFeatures (curr_features);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        align.align (*aligned);
    }   

    if (align.hasConverged ())
    {
        Mat R, t;
        Eigen::Matrix4f transformation; 
        align.computeTransformation (*aligned,transformation);

        ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        ROS_INFO("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        ROS_INFO("\n");
        ROS_INFO("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        ROS_INFO("\n");
        ROS_INFO_STREAM("Inliers:" << align.getInliers ().size () << currNT->size ());
        
        //R = transformation(Rect(0,0,2,2));
        //t = transformation(Rect(0,3,2,3));
    
        if(state == init)
        {
            R_f = R.clone();
            t_f = t.clone();
            state = processing;
        }
        else
        {
            t_f = t_f + (R_f*t);
            R_f = R*R_f;
        }
    }
    else
    {
        ROS_INFO_STREAM("couldn't align");
    }*/

    prev = curr;
    prev_features = curr_features;
            
    /*
    
    int myx = int(t_f.at<double>(0));
    int myz = int(t_f.at<double>(2));

    ROS_INFO_STREAM("*X = " << myx << " Y = " << myz);*/
}

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "optical_flow_node");
    ros::NodeHandle nh;
   
    ros::Subscriber groundPlaneEdge = nh.subscribe("/camera/depth_registered/points", 1, callback);    

    ros::spin();    
       
    return 0;
}
