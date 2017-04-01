

#ifndef _PARTICLEFILTER_H_
#define _PARTICLEFILTER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <list>
#include "corobot_common/Pose.h"



using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;
using corobot_common::Pose;
using sensor_msgs::LaserScan;

typedef struct {
   int particleId;
   Pose pose;
   uint32_t poseXpixels;  // Map position in pixels
   uint32_t poseYpixels;  // Map position in pixels
   double weight;
} Particle;



class ParticleFilter
{
   public:
      
      typedef std::list<Particle> ParticleList;
   
      ParticleFilter(OccupancyGrid & map, float resamplePercentage, int orientationRangeDeg, int numLaserScans);
   
      virtual ~ParticleFilter();
      
      bool initialize (int numParticles);
     
      int getNumParticles() {return mNumParticles;}
      
      ParticleList& getParticleList() {return mParticles;};
      
      void updateParticlePositions(Odometry odom);
      
      void updateParticleSensorData(LaserScan& scan);
      
      Pose calculatePosition();
      
      void setqrCodePose(Pose currentQRCodePose);
      
   private:
   
      // This is a port of odom_to_pose in utils.py
      Pose OdomToPose(Odometry& odom);
      
      // This function handles the wrapping around of a unit circle angle in radians.
      // The function name is from matlab.
      float wrapTo2Pi(float theta);
      
      // This function the repopulates the pool of particles particles when the 
      // population drops below a certain threshold.
      void resample();
      
      // This is a port from  CODE GENIUS - Drawing Lines with Bresenham's Line Algorithm by Jenn Schiffer
      // https://www.youtube.com/watch?v=zytBpLlSHms
      bool bresenheim(uint32_t x1pixel, uint32_t y1pixel, uint32_t& x2pixel, uint32_t& y2pixel);
      
      // This is a port from corobot-kinetic\corobot_localization\src\bresenheim.py
      bool bresenheimCondition(int xpixel, int ypixel);
      
      // Get laser probability.
      // this is based upon the code corobot-kinetic\corobot_localization\src\kinect_loc.py
      // Handles the error ranges for the laser.
      double getLaserProbability(float observedm, float expectedm);
      
      // this is a utility function that will convert from cartesian to the grid
      // coordinate system.  
      void cartesianToGrid(double xm, double ym, uint32_t& xpixel, uint32_t& ypixel);
      
      // this is a utility function that will convert from cartesian to the grid
      // coordinate system.  
      void gridToCartesian(uint32_t xpixel, uint32_t ypixel, double& xm, double& ym);
      
      Pose mLastPose;
      
      // the origin of the grid map matches cartesian coordinates. This does not match the map in 
      // any other part of the system.
      OccupancyGrid mMap;
      
      
      float mMapWidthm;
      float mMapHeightm;

      ParticleList mParticles;
      
      int mNumOpenSpaces;
      int mNumParticles;
      
      int mNumLaserScans;
      int* mLaserScanRangeIndex;
      float *mLaserSanRangeAngleRad;
      float mMinWeight;
      float mMaxWeight;
      
      
      int mOrientationRangeDeg;
      float mResamplePercentage;
      unsigned int mResampleThreshold;
      
      uint32_t mRoombaRadiusPixels;
      bool mInitialized; 
};


#endif