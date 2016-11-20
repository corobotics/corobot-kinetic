#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include <pcl/PCLHeader.h>
#include <pcl/PointIndices.h>
#include<cmath>

enum DrivingDirection {
    LEFT,
    RIGHT,
    FORWARD
};

class ObstacleDetection {

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher groundDetect;
//    ros::Publisher occlusion;
    DrivingDirection direction;  //direction of movement of robot
    DrivingDirection previousTurn; //previous move of the robot
    double previous_farY, previous_nearY,
            previous_farZ, previous_nearZ;

    double slope, intercept;


public:

    ObstacleDetection(ros::NodeHandle nh) :
            nodeHandle(nh),
            groundDetect(nodeHandle.advertise<sensor_msgs::PointCloud2>("groundPlane", 1)),
//            occlusion(nodeHandle.advertise<sensor_msgs::PointCloud2>("navigation", 1)),
            direction(FORWARD), previous_farY(0), previous_nearY(0), previous_farZ(0), previous_nearZ(0) {
        ROS_INFO("Inside constructor ..");
        ros::Subscriber groundPlaneEdge = nodeHandle.subscribe("/camera/depth_registered/points", 1,
                                                               &ObstacleDetection::groundPlaneDetection, this);
        ROS_INFO("Subscribed to PointCloud2 topic...");
        ros::spin();
    }

    void groundPlaneDetection(const sensor_msgs::PointCloud2ConstPtr &input) {

        ROS_INFO("Method call successful...");
        sensor_msgs::PointCloud2 cloud_pointer;
        sensor_msgs::PointCloud2 nav_pointer;
        pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2 cloud_filtered;

        pcl_conversions::toPCL(*input, *cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2::Ptr points(new pcl::PCLPointCloud2);

        pcl::PCLPointCloud2 *nav = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2 nav_filtered;

        pcl::PassThrough <pcl::PCLPointCloud2> passThrough;
        pcl::RadiusOutlierRemoval <pcl::PCLPointCloud2> outlierRemoval;
        double distanceFromGround = 0.0;

        double radiusAlongX, maxCropAlongZ, minCropAlongZ,
                maxCropAlongY, minCropAlongY,
                minThreshold, maxThreshold,
                lateralBumper, frontalBumper,
                normalSmoothing, outlierRadius,
                fineToleranceLevel, roughToleranceLevel,
                closeY, farY, closeZ, farZ;

        int noOfOutliers = 6;

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


        ROS_INFO("Calculating slope and intercept");

        if (closeY != previous_nearY || farY != previous_farY || closeZ != previous_nearZ || farZ != previous_farZ) {

            double diffY, diffZ, sumY, sumZ;
            diffY = farY - closeY;
            diffZ = farZ - closeZ;
            sumY = closeY + farY;
            sumZ = closeZ + farZ;
	
	    ROS_INFO("Calculating slope and intercept");
            slope = diffY/diffZ;
            intercept = sumY/2 - slope * sumZ / 2;

            previous_nearY = closeY;
            previous_farY = farY;
            previous_nearZ = closeZ;
            previous_farZ = farZ;
      
            ROS_INFO("Slope %f", slope);
            ROS_INFO("Intercept %f", intercept);
        }

        ROS_INFO("Passing cloud through passthrough filter");

        //Set the frame of reference on the ground points for X, Y and Z axis

        passThrough.setInputCloud(cloudPtr);
        passThrough.setFilterFieldName("x");
        passThrough.setFilterLimits(-radiusAlongX - lateralBumper, radiusAlongX + lateralBumper);
        passThrough.setKeepOrganized(true);
        passThrough.filter(*points);

        passThrough.setInputCloud(points);
        passThrough.setFilterFieldName("y");
        passThrough.setFilterLimits(minCropAlongY, maxCropAlongY);
        passThrough.setKeepOrganized(true);
        passThrough.filter(*points);

        passThrough.setInputCloud(points);
        passThrough.setFilterFieldName("z");
        passThrough.setFilterLimits(minCropAlongZ, maxCropAlongZ + frontalBumper);
        passThrough.setKeepOrganized(true);
        passThrough.filter(*points);

        passThrough.filter(cloud_filtered);

	double count = 0.0;
        ROS_INFO("Calculating ground points");

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*points, *temp_cloud);

        for (pcl::PointCloud<pcl::PointXYZ>::iterator
                     location = temp_cloud->begin(); location < temp_cloud->end(); location++) {

//            ROS_INFO("x-coord %f", location->x);
//           ROS_INFO("y-coord %f", location->y);
            //ROS_INFO("Location cloud");

            double expectedYCoord = slope * location->z + intercept;
            distanceFromGround = fabs(location->y - expectedYCoord);
//            ROS_INFO("Distance from Ground %f", distanceFromGround);

	    count+=distanceFromGround;
            if (distanceFromGround > roughToleranceLevel && !std::isnan(distanceFromGround)) {
                location->x = std::numeric_limits<float>::quiet_NaN();
                location->y = std::numeric_limits<float>::quiet_NaN();
                location->z = std::numeric_limits<float>::quiet_NaN();

            }
                // if the point detected is close to the ground
            else if (distanceFromGround >= fineToleranceLevel && distanceFromGround <= roughToleranceLevel && !std::isnan(distanceFromGround))
//                     fabs(location->x) < (radiusAlongX - lateralBumper) &&
//                    location->z > (closeZ + frontalBumper) &&
//                   location->z < (maxCropAlongZ - frontalBumper)) {
   	    {             //actual ground plane we are interested in
                groundPoints++;
                totalGroundPoints += location->x;
            }
        }

	ROS_INFO("Distance from Ground %f", count);
        ROS_INFO("Ground Points : %d", groundPoints);
	ROS_INFO("Total Ground Points : %f", totalGroundPoints);


        if (groundPoints <= 0) {
            isGround = false;
        }

        /*
        //find edges and eliminate the outliers from the ground plane detected.
        if (groundPoints > 0) {


            pcl::PointCloud <pcl::PointXYZ> nav_input_cloud;
            nav_input_cloud.header = cloud_filtered.header;

            pcl::toPCLPointCloud2(*temp_cloud, cloud_filtered);
            pcl::PointCloud <pcl::PointXYZ> input_cloud;
            pcl::fromPCLPointCloud2(cloud_filtered, input_cloud);

            ROS_INFO("Iterating through navigation cloud...");
            for (std::vector<pcl::PointIndices>::iterator
                         edge = edges.begin(); edge < edges.end(); edge++) {

                for (std::vector<int>::iterator
                             point = edge->indices.begin(); point < edge->indices.end(); point++) {

                    if (fabs((input_cloud)[*point].x) < radiusAlongX - lateralBumper &&
                        (input_cloud)[*point].z > closeZ + frontalBumper &&
                        (input_cloud)[*point].z < farZ - frontalBumper) {
                        nav_input_cloud.push_back((input_cloud)[*point]);
                    }
                }
            }

            //sensor_msgs::PointCloud2ConstPtr& temp_nav;
            pcl::fromROSMsg(nav_pointer, nav_input_cloud);
            pcl_conversions::toPCL(nav_pointer, *nav);
            pcl::PCLPointCloud2ConstPtr cloud_nav(nav);

            // eliminate the outliers from the frame
            if (outlierRadius >= 0 && cloud_nav->height * cloud_nav->width > 0) {
                outlierRemoval.setInputCloud(cloud_nav);
                outlierRemoval.setRadiusSearch((float) outlierRadius);
                if (noOfOutliers >= 0) {
                    outlierRemoval.setMinNeighborsInRadius(noOfOutliers);
                }
                outlierRemoval.filter(nav_filtered);
            }
        } else if (isGround) {
            ROS_INFO("Edges::Cannot detect the ground %d", isGround);
        }
        */

        if (groundPoints > 0) {
            if (outlierRadius >= 0 && points->height * points->width > 0) {
                outlierRemoval.setInputCloud(points);
                outlierRemoval.setRadiusSearch((float) outlierRadius);
                if (noOfOutliers >= 0) {
                    outlierRemoval.setMinNeighborsInRadius(noOfOutliers);
                }
                outlierRemoval.filter(cloud_filtered);
            }
        } else {
            ROS_INFO("Cannot detect the ground %d", isGround);
        }

        pcl::PointCloud <pcl::PointXYZ> cloudForNav;
        pcl::fromPCLPointCloud2(cloud_filtered, cloudForNav);

        //to determine the direction of the movement for robot

        if (cloudForNav.size() > 0) {
            float centroidOfX = 0.0;

            for (pcl::PointCloud<pcl::PointXYZ>::iterator
                         point = cloudForNav.begin(); point < cloudForNav.end(); point++) {
                centroidOfX += point->x;
            }

	    ROS_INFO("Centroid : %f", centroidOfX);

            centroidOfX /= cloudForNav.size();

            if (centroidOfX < 0) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }

            previousTurn = direction;

        } else if (groundPoints == 0) { //cannot find ground
            if (previousTurn == LEFT) {
                direction = RIGHT;
            } else if (previousTurn == RIGHT) {
                direction = LEFT;
            }

        } else if(groundPoints>0){
            direction = FORWARD;
            previousTurn = totalGroundPoints / groundPoints > 0 ? RIGHT : LEFT;
        }

        if (direction == LEFT) {
            ROS_INFO("Turn Left");
        } else if (direction == RIGHT) {
            ROS_INFO("Turn Right");
        } else {
            ROS_INFO("Move Forward");
        }

        pcl_conversions::fromPCL(cloud_filtered, cloud_pointer);
        groundDetect.publish(cloud_pointer);
        //       occlusion.publish(nav_pointer);
    }
};


int main(int argc, char **argv) {

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
    node.setParam("fineToleranceLevel", 0.2);
    node.setParam("roughToleranceLevel", 0.25);
    node.setParam("closeY", 0.3);
    node.setParam("farY", 0.47);
    node.setParam("closeZ", 0.8);
    node.setParam("farZ", 2.5);
    node.setParam("frontalBumper", 0.1);
    node.setParam("lateralBumper", 0.02);
    node.setParam("outlierRadius", 0.05);
    node.setParam("isGround", false);

    ROS_INFO("Calling constructor");
    ObstacleDetection obstacleDetect(node);

    return (0);

}
