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

//Mat R_f,t_f;

/*void findCorrespondences (FeatureCloudT::Ptr source, FeatureCloudT::Ptr target, std::vector<int>& correspondences) 
{
	ROS_INFO_STREAM("1.1");
	correspondences.resize (source->size());
	ROS_INFO_STREAM("1.2");
  	// Use a KdTree to search for the nearest matches in feature space
  	KdTreeFLANN<FeatureT> descriptor_kdtree;
  	descriptor_kdtree.setInputCloud (target);
ROS_INFO_STREAM("1.3");
  	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
  	const int k = 1;
  	for (int i = 0; i < static_cast<int> (source->size ()); ++i)
  	{
		std::vector<int> k_indices (k);
  		std::vector<float> k_squared_distances (k);
		ROS_INFO_STREAM("1.31");  		
		descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
		ROS_INFO_STREAM("1.32");
    	correspondences[i] = k_indices[0];
  	}
ROS_INFO_STREAM("1.44");
}

void filterCorrespondences (KeyPointCloudT::Ptr& source_keypoints_, KeyPointCloudT::Ptr& target_keypoints_,std::vector<int>& target2source_, std::vector<int>& source2target_, CorrespondencesPtr& correspondences_)
{
	ROS_INFO_STREAM("0");
    
  	std::vector<std::pair<unsigned, unsigned> > correspondences;
  	for (unsigned cIdx = 0; cIdx < source2target_.size (); ++cIdx)
    	if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
      		correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));
ROS_INFO_STREAM("1");
    
  	correspondences_->resize (correspondences.size());
  	for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  	{
    	(*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    	(*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  	}
ROS_INFO_STREAM("2");
    
  	registration::CorrespondenceRejectorSampleConsensus<KeyPointT> rejector;
  	rejector.setInputSource (source_keypoints_);
  	rejector.setInputTarget (target_keypoints_);
  	rejector.setInputCorrespondences(correspondences_);
  	rejector.getCorrespondences(*correspondences_);
ROS_INFO_STREAM("4");
    
}*/
/*
void determineInitialTransformation ()
{
  	registration::TransformationEstimation<KeyPointT, KeyPointT>::Ptr transformation_estimation (new registration::TransformationEstimationSVD<KeyPointT, KeyPointT>);

  	transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

  	pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
}

void determineFinalTransformation ()
{
  	pcl::Registration<PointXYZRGB, PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<PointXYZRGB, PointXYZRGB>);
  	registration->setInputSource (source_transformed_);
  	//registration->setInputSource (source_segmented_);
  	registration->setInputTarget (target_segmented_);
  	registration->setMaxCorrespondenceDistance(0.05);
  	registration->setRANSACOutlierRejectionThreshold (0.05);
  	registration->setTransformationEpsilon (0.000001);
  	registration->setMaximumIterations (1000);
  	registration->align(*source_registered_);
  	transformation_matrix_ = registration->getFinalTransformation();
}
*/

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
    
    ROS_INFO_STREAM("Converting data, removing NaN");
    
    fromROSMsg(*pointer, *curr);
    
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*curr, *curr, indices);
    
	ROS_INFO_STREAM("Estimating Normals");

    // Apply filter (noise removal) on the scene
    

    // Estimate normals for scene
    /*IntegralImageNormalEstimation<PointIn, PointN> nest;
    nest.setNormalEstimationMethod (nest.AVERAGE_3D_GRADIENT);
    nest.setMaxDepthChangeFactor(0.02f);
    nest.setNormalSmoothingSize(10.0f);
    nest.setInputCloud(curr);
    nest.compute(*currN);    */

	NormalEstimation<PointIn, PointN> normal_estimation;
    normal_estimation.setSearchMethod (search::Search<PointIn>::Ptr (new search::KdTree<PointIn>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (curr);
	normal_estimation.compute (*currN);

    //ROS_INFO_STREAM("1:" << curr->size ());
    //ROS_INFO_STREAM("2:" << currN->size ());

    ROS_INFO_STREAM("Finding keypoints");

    // Find  keypoints   
    /*HarrisKeypoint3D<PointIn,KeyPointT> hkest;
	hkest.setMethod(HarrisKeypoint3D<PointIn,KeyPointT>::TOMASI);
	hkest.setNonMaxSupression (true);
    hkest.setRadius (0.01f);
	hkest.setRadiusSearch (0.01f);
    hkest.setInputCloud(curr);
    hkest.compute(*curr_KeyPoints);*/

	SIFTKeypoint<PointIn, PointT> sift3D;
    sift3D.setScales (0.01f, 3, 2);
    sift3D.setMinimumContrast (0.0);
	sift3D.setInputCloud(curr);
    sift3D.compute(*curr_KeyPoints);

	PointCloudIn::Ptr kpts(new PointCloudIn);
  	kpts->points.resize(curr_KeyPoints->points.size());

	pcl::copyPointCloud(*curr_KeyPoints, *kpts);

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
    ROS_INFO("\n");    

	prev = curr;
    prev_features = curr_features;
	prev_KeyPoints = curr_KeyPoints;
            
    /*
    
    int myx = int(t_f.at<double>(0));
    int myz = int(t_f.at<double>(2));

    ROS_INFO_STREAM("*X = " << myx << " Y = " << myz);*/


	pcl::visualization::PCLVisualizer vis;
    //add the first cloud to the viewer
    vis.addPointCloud (prev->makeShared(), "src_points");
       
    //transfor the second cloud to be able to view them without overlaying each other
    Eigen::Matrix4f t;
    t<<1,0,0,50,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;
    pcl::PointCloud<pcl::PointXYZRGB> cloudview;
    pcl::transformPointCloud(*curr,cloudview,t);

    //add the second point cloud
    vis.addPointCloud (cloudview.makeShared(), "tgt_points");

    bool alter=false;
               
    //copy the cloudNext Keypoints to keypointsDisplay to prevent altering the data in p_tgt.x+=50;
    pcl::PointCloud<PointT> keypointDisplay;
    pcl::copyPointCloud<PointT> (*curr_KeyPoints,keypointDisplay);

	for (size_t i =0; i <final_correspondences->size (); i++)
    { 
		PointT & p_src = (*prev_KeyPoints).points.at((*final_correspondences)[i].index_query);
		PointT & p_tgt = keypointDisplay.points.at((*final_correspondences)[i].index_match);

	 
		p_tgt.x+=50;
		   

		// Draw the line
		std::stringstream ss ("line");
		ss << i;
		std::stringstream sss ("spheresource");
		sss << i;
		std::stringstream ssss ("spheretarget");
		ssss << i;
		if(alter)
		{
			vis.addSphere(p_src,0.5,255,0,0,sss.str());
		    vis.addSphere(p_tgt,0.5,255,0,0,ssss.str());
		    vis.addLine (p_src, p_tgt, 0, 0, 255, ss.str ());       
		}
		else
		{
		    vis.addSphere(p_src,0.5,255,255,0,sss.str());
		    vis.addSphere(p_tgt,0.5,255,255,0,ssss.str());
		  	vis.addLine (p_src, p_tgt, 220, 24, 225, ss.str ());
		}
		alter != alter;
	}
   
    vis.resetCamera ();
    vis.spin ();  
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
