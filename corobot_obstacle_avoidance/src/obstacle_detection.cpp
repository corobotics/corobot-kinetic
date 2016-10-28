#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/features/organized_edge_detection.h"
#include "pcl/features/integral_image_normal.h"
#include <pcl/PCLHeader.h>

enum DrivingDirection {
    LEFT,
    RIGHT,
    FORWARD
};

class ObstacleDetection {

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher groundDetect;
    ros::Publisher occlusion;
    DrivingDirection direction;  //direction of movement of robot
    DrivingDirection previousTurn; //previous move of the robot
    double previous_farY,previous_nearY,
            previous_farZ, previous_nearZ;

    double slope, intercept;

public:

    ObstacleDetection(ros::NodeHandle nh):
            nodeHandle(nh),
            groundDetect(nodeHandle.advertise<sensor_msgs::PointCloud2>("groundPlane",1)),
            direction(FORWARD), previous_farY(0), previous_nearY(0), previous_farZ(0), previous_nearZ(0) {
	//ROS_INFO("Inside constructor ...");
        ros::Subscriber groundPlaneEdge = nodeHandle.subscribe("/camera/depth/points", 1, &ObstacleDetection::groundPlaneDetection,this);
        ros::spin();
    }

    void groundPlaneDetection(const sensor_msgs::PointCloud2ConstPtr& input){

        sensor_msgs::PointCloud2 cloud_pointer;
        sensor_msgs::PointCloud2 nav_pointer;
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered;

        pcl_conversions::toPCL(*input,*cloud);
        pcl::PCLPointCloud2* points = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloud_points(points);

        pcl::PCLPointCloud2* nav = new pcl::PCLPointCloud2;        
	//pcl::PCLPointCloud2ConstPtr cloud_nav(nav);
        pcl::PCLPointCloud2 nav_filtered;

        pcl::PassThrough<pcl::PCLPointCloud2> passThrough;
        pcl::OrganizedEdgeFromRGBNormals<pcl::PCLPointCloud2, pcl::Normal, pcl::Label> edgeDetection;
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outlierRemoval ;

        pcl::PointCloud<pcl::Label> edgePoints;
        std::vector<pcl::PointIndices> edges;

        double radiusAlongX, maxCropAlongZ, minCropAlongZ,
                maxCropAlongY, minCropAlongY,
                minThreshold, maxThreshold,
                lateralBumper, frontalBumper,
                normalSmoothing, outlierRadius,
                fineToleranceLevel, roughToleranceLevel,
                closeY, farY, closeZ, farZ;

        int noOfOutliers, normalEstimation;

        int groundPoints = 0;
        double totalGroundPoints = 0;

        bool isGround;

        nodeHandle.getParamCached("radiusAlongX", radiusAlongX);
        nodeHandle.getParamCached("maxCropAlongZ", maxCropAlongZ);
        nodeHandle.getParamCached("minCropAlongZ", minCropAlongZ);
        nodeHandle.getParamCached("maxCropAlongY", maxCropAlongY);
        nodeHandle.getParamCached("minCropAlongY", minCropAlongY);
        nodeHandle.getParamCached("minThreshold", minThreshold);
        nodeHandle.getParamCached("maxThreshold", maxThreshold);
        nodeHandle.getParamCached("fineToleranceLevel", fineToleranceLevel);
        nodeHandle.getParamCached("roughToleranceLevel", roughToleranceLevel);
        nodeHandle.getParamCached("closeY", closeY);
        nodeHandle.getParamCached("farY", farY);
        nodeHandle.getParamCached("closeZ", closeZ);
        nodeHandle.getParamCached("farZ", farZ);
        nodeHandle.getParamCached("frontalBumper", frontalBumper);
        nodeHandle.getParamCached("lateralBumper", lateralBumper);
        nodeHandle.getParamCached("normalSmoothing", normalSmoothing);
        nodeHandle.getParamCached("outlierRadius", outlierRadius);
        nodeHandle.getParamCached("isGround", isGround);


        if(closeY!=previous_nearY || farY != previous_farY || closeZ!=previous_nearZ || farZ!=previous_farZ){
            double diffY, diffZ, sumY, sumZ;
            diffY = farY - closeY;
            diffZ = farZ - closeZ;
            sumY = closeY + farY;
            sumZ = closeZ + farZ;

            slope = diffY/diffZ;
            intercept = sumY/2 - slope * sumZ / 2;

            previous_nearY = closeY;
            previous_farY = farY;
            previous_nearZ = closeZ;
            previous_farZ = farZ;
        }

        //Set the frame of reference on the ground points for X, Y and Z axis
        passThrough.setInputCloud(cloudPtr);
        passThrough.setFilterFieldName("x");
        passThrough.setFilterLimits(-radiusAlongX - lateralBumper,radiusAlongX + lateralBumper);
        passThrough.setKeepOrganized(true);
        passThrough.filter(cloud_filtered);

        passThrough.setInputCloud(cloud_points);
        passThrough.setFilterFieldName("y");
        passThrough.setFilterLimits(maxCropAlongY,1.0);
        passThrough.setKeepOrganized(true);
        passThrough.filter(cloud_filtered);

        passThrough.setInputCloud(cloud_points);
        passThrough.setFilterFieldName("z");
        passThrough.setFilterLimits(minCropAlongZ, maxCropAlongZ + frontalBumper);
        passThrough.setKeepOrganized(true);
        passThrough.filter(cloud_filtered);

//	pcl::PointCloud<PointT> pointCloud;
//      pcl::toPCLPointCloud2(pointCloud, cloud_points);
	pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromPCLPointCloud2(cloud_filtered, input_cloud);		
        for(pcl::PointCloud<pcl::PointXYZ>::iterator
                    location = input_cloud.begin(); location < input_cloud.end(); location++){

            double expectedYCoord = slope *  location->z + intercept;
            double distanceFromGround = fabs(location->y - expectedYCoord);

            // if the point detected is not anywhere close to the ground
            if(distanceFromGround>roughToleranceLevel){
                location->x = std::numeric_limits<float>::quiet_NaN();
                location->y = std::numeric_limits<float>::quiet_NaN();
                location->z = std::numeric_limits<float>::quiet_NaN();
            }
            else if( distanceFromGround<fineToleranceLevel &&
                    fabs(location->x) < (radiusAlongX - lateralBumper) &&
                    location->z > (closeZ + frontalBumper) &&
                    location->z < (maxCropAlongZ-frontalBumper) ){
               //actual ground plane we are interested in
                groundPoints++;
                totalGroundPoints+= location->x;
            }
        }

        //find edges and eliminate the outliers from the ground plane detected.
        if(groundPoints>0){
            edgeDetection.setInputCloud(cloud_points);
            edgeDetection.setEdgeType(edgeDetection.EDGELABEL_HIGH_CURVATURE +
                                      edgeDetection.EDGELABEL_NAN_BOUNDARY );

//            if(normalEstimation>=0){
//                edgeDetection.setHighCurvatureNormalEstimationMethod(
//                        (pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod) normalEstimation);
//            }
//
//            if(normalSmoothing>=0){
//                edgeDetection.setHighCurvatureNormalSmoothingSize((float)normalSmoothing);
//            }

            if(minThreshold>=0){
                edgeDetection.setHCCannyLowThreshold((float)minThreshold);
            }
            if(maxThreshold>=0){
                edgeDetection.setHCCannyHighThreshold((float)maxThreshold);
            }
	  

	    pcl::PointCloud<pcl::PointXYZ> nav_input_cloud;
	    nav_input_cloud.header = cloud_points->header;

            for(std::vector<pcl::PointIndices>:: iterator
                    edge=edges.begin(); edge < edges.end(); edge++){

                for(std::vector<int>::iterator
                            point=edge->indices.begin(); point< edge->indices.end(); point++){

                    if( fabs((input_cloud)[*point].x) < radiusAlongX-lateralBumper &&
                            (input_cloud)[*point].z > closeZ + frontalBumper &&
                            (input_cloud)[*point].z < farZ - frontalBumper ){
                        nav_input_cloud.push_back((input_cloud)[*point]);
                    }
                }
            }
		
	   //sensor_msgs::PointCloud2ConstPtr& temp_nav;
    	    pcl::fromROSMsg(nav_pointer, nav_input_cloud);   
	    pcl_conversions::toPCL(nav_pointer, *nav);
	    pcl::PCLPointCloud2ConstPtr cloud_nav(nav);

	    // eliminate the outliers from the frame
            if(outlierRadius>=0 && cloud_nav->height*cloud_nav->width > 0){
                outlierRemoval.setInputCloud(cloud_nav);
                outlierRemoval.setRadiusSearch((float) outlierRadius);
                if(noOfOutliers>=0){
                    outlierRemoval.setMinNeighborsInRadius(noOfOutliers);
                }
                outlierRemoval.filter(nav_filtered);
            }
        }
        else if(isGround){
            ROS_INFO("Edges::Cannot detect the ground %d", isGround);
        }

	pcl::PointCloud<pcl::PointXYZ> nav_filtered_cloud;
        pcl::fromPCLPointCloud2(nav_filtered, nav_filtered_cloud);

        //to determine the direction of the movement for robot
        if(nav_filtered_cloud.size() > 0){
            float centroidOfX = 0;

            for(pcl::PointCloud<pcl::PointXYZ>:: iterator
                    point=nav_filtered_cloud.begin(); point< nav_filtered_cloud.end(); point++){
                centroidOfX+=point->x;
            }

            centroidOfX/=nav_filtered_cloud.size();

            if(centroidOfX<0){
                direction = RIGHT;
            }
            else{
                direction = LEFT;
            }
            previousTurn = direction;
        }
        else if(groundPoints==0){ //cannot find ground
            if(previousTurn==LEFT){
                direction=RIGHT;
            } else if(previousTurn==RIGHT){
                direction=LEFT;
            }
        }
        else{
            direction=FORWARD;
            previousTurn = totalGroundPoints/groundPoints > 0 ? RIGHT : LEFT;
        }

        pcl_conversions::fromPCL(cloud_filtered, cloud_pointer);
        pcl_conversions::fromPCL(nav_filtered, nav_pointer);
        groundDetect.publish(cloud_pointer);
        occlusion.publish(nav_pointer);
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "corobot_obstacle_detection");
    ros::NodeHandle node("groundPlaneDetection");

    // set nodeHandle params
    node.setParam("radiusAlongX", 0.5);
    node.setParam("maxCropAlongZ", 1.0);
    node.setParam("minCropAlongZ", 0.0);
    node.setParam("maxCropAlongY", 1.0);
    node.setParam("minCropAlongY", -1.0);
    node.setParam("minThreshold", 1.0);
    node.setParam("maxThreshold", 2.0);
    node.setParam("fineToleranceLevel", 0.0);
    node.setParam("roughToleranceLevel", 0.5);
    node.setParam("closeY", 0.5);
    node.setParam("farY", 1.0);
    node.setParam("closeZ", 0.0);
    node.setParam("farZ", 1.0);
    node.setParam("frontalBumper", 0.1);
    node.setParam("lateralBumper", 0.01);
    node.setParam("normalSmoothing", -1.0);
    node.setParam("outlierRadius", 0.05);
    node.setParam("isGround", true);

    //ROS_INFO("Calling constructor");
    ObstacleDetection obstacleDetect(node);

    return (0);

}
