#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>

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

using namespace cv;
using namespace pcl;

// Types
typedef PointXYZRGB PointIn;
typedef PointCloud<PointIn> PointCloudIn;

typedef PointXYZI PointT;
typedef PointCloud<PointT> PointCloudT;

typedef Normal PointN;
typedef PointCloud<PointN> PointCloudN;

typedef PointXYZI KeyPointT;
typedef PointCloud<KeyPointT> KeyPointCloudT;

typedef PFHSignature125 FeatureT;
typedef PointCloud<FeatureT> FeatureCloudT;

enum States
{
    boot = 0,
    init,
    processing
};

States state = boot;

PointCloudIn::Ptr prev;
FeatureCloudT::Ptr prev_features;  
KeyPointCloudT::Ptr prev_KeyPoints;

int count = 0;
//Mat R_f,t_f;


// Handler / callback
void callback(const sensor_msgs::PointCloud2ConstPtr& pointer)
{
	ROS_INFO_STREAM("Received callback");
   
    PointCloudIn::Ptr curr (new PointCloudIn);

    //PointCloudT::Ptr curr (new PointCloudT);
	KeyPointCloudT::Ptr aligned (new KeyPointCloudT);
    PointCloudN::Ptr currN(new PointCloudN);

    KeyPointCloudT::Ptr curr_KeyPoints (new KeyPointCloudT);

	FeatureCloudT::Ptr curr_features (new FeatureCloudT);
    
    
    fromROSMsg(*pointer, *curr);

	/*char text[20];
	sprintf(text,"test_pcd%d.pcd", count);
	count++;
	pcl::io::savePCDFileASCII (text, *curr);
 	ROS_INFO_STREAM("Saving file");*/
 
	// Create the filtering object
	StatisticalOutlierRemoval<PointIn> sor;
	sor.setInputCloud (curr);
	sor.setMeanK (10);
	sor.setStddevMulThresh (1.0);
	sor.filter (*curr); 	
 
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*curr, *curr, indices);
    
	ROS_INFO_STREAM("Estimating Normals");
    // Estimate normals for scene
	NormalEstimation<PointIn, PointN> normal_estimation;
    normal_estimation.setSearchMethod (search::Search<PointIn>::Ptr (new search::KdTree<PointIn>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (curr);
	normal_estimation.compute (*currN);

    ROS_INFO_STREAM("Finding keypoints");
    // Find  keypoints   
	SIFTKeypoint<PointIn, PointT> sift3D;
    sift3D.setScales (0.05, 4, 5);
    sift3D.setMinimumContrast (1.0);
	sift3D.setInputCloud(curr);
    sift3D.compute(*curr_KeyPoints);

	PointCloudIn::Ptr kpts(new PointCloudIn);
  	kpts->points.resize(curr_KeyPoints->points.size());

	pcl::copyPointCloud(*curr_KeyPoints, *kpts);

	ROS_INFO_STREAM(curr_KeyPoints->points.size());
	ROS_INFO_STREAM("Estimating features for keypoints");

    // Estimate features for keypoints 
    PFHEstimation<PointIn,Normal,FeatureT> pest;
 	pest.setKSearch(50);
	pest.setInputNormals(currN);
   	pest.setSearchSurface(curr);    
	pest.setInputCloud (kpts);
	pest.compute (*curr_features);
    
    // if this is first image store it and go to next iteration
    if(state == boot)    
    {
		prev = curr; 
        prev_features = curr_features;
		prev_KeyPoints = curr_KeyPoints;
        state = init;        
        return;
    }    
        
	//std::vector<int> source2target;
	//std::vector<int> target2source;
	//findCorrespondences(prev_features,curr_features,source2target);
	//findCorrespondences(curr_features,prev_features,target2source);

	ROS_INFO_STREAM("Finding intial correspondance.");

	ROS_INFO_STREAM(":" << prev_features->size ());
    ROS_INFO_STREAM(":" << curr_features->size ());
 	
	// ... read or fill in source and target
	CorrespondencesPtr intial_correspondences(new pcl::Correspondences);
	registration::CorrespondenceEstimation< FeatureT, FeatureT> est;
	est.setInputSource (prev_features);
	est.setInputTarget (curr_features);
	est.determineReciprocalCorrespondences (*intial_correspondences);

	ROS_INFO_STREAM("Finding final correspondance.");

	CorrespondencesPtr final_correspondences(new pcl::Correspondences);
  	registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
  	rejector.setInputSource (prev_KeyPoints);
  	rejector.setInputTarget (curr_KeyPoints);
  	rejector.setInputCorrespondences(intial_correspondences);
  	rejector.getCorrespondences(*final_correspondences);	

	ROS_INFO_STREAM("Finding Transformation matrix.");

	Eigen::Matrix4f transformation;
	registration::TransformationEstimationSVD<PointT,PointT> transformation_estimation;
	transformation_estimation.estimateRigidTransformation(*prev_KeyPoints, *curr_KeyPoints, *final_correspondences, transformation);

	ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    ROS_INFO("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    ROS_INFO("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    ROS_INFO("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));    

	prev = curr;
    prev_features = curr_features;
	prev_KeyPoints = curr_KeyPoints; 

	pcl::visualization::PCLVisualizer vis;
    //add the first cloud to the viewer
    vis.addPointCloud (prev->makeShared(), "src_points");
       
    //transfor the second cloud to be able to view them without overlaying each other
    Eigen::Matrix4f t;
    t<<1,0,0,5,
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
	pcl::PointCloud<PointT> keypointDisplay;
    pcl::copyPointCloud<PointT> (*curr_KeyPoints,keypointDisplay);
	ROS_INFO("Adding correspondance");
    

	for (size_t i =0; i <final_correspondences->size (); i++)
    { 
		PointT & p_src = (*prev_KeyPoints).points.at((*final_correspondences)[i].index_query);
		PointT & p_tgt = keypointDisplay.points.at((*final_correspondences)[i].index_match);

	 
		p_tgt.x+=5;
		   

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
	ROS_INFO("Complete");
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
