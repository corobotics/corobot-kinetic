/**
 * \class ParticleFilter
 *
 * This class implements the internals of the particle filter.  It sets down a 
 * field of particles.  Processing involves updating the field of particles 
 *
 */

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
   int particleId;        // Particle ID for debugging purposes.
   Pose pose;             // Robots positon in meters. 
   uint32_t poseXpixels;  // Robots Map position in pixels
   uint32_t poseYpixels;  // Robots Map position in pixels
   double weight;         // 0..1 range.
} Particle;



class ParticleFilter
{
   public:
      
      // Holds our particles 
      typedef std::list<Particle> ParticleList;
   
      /** 
       *  @Function Constructor
       ^ 
       *  @param map the robots occupancy grid.
       *  @param resamplePercentage. a value from 0..1. 
       *  @param orientationRangeDeg. Sets a limitatio on the random orientation of a new particle during repopulation. 
       *  @param numLaserScans number of laser scans to process in the corobots sensor data.
       *
       */
       ParticleFilter(OccupancyGrid & map, float resamplePercentage, int orientationRangeDeg, int numLaserScans);
   
      /** 
       * @Function Destructor

       */
      virtual ~ParticleFilter();
      
      /** 
       *  @Function initialize
       ^ 
       *  @param numParticles the number of particles you want in the map.
       *
       *  @ return true if initialized
       */
       
      bool initialize (int numParticles);
     
       /** 
       *  @Function getNumParticle.
       *
       *  @ return number of particles in the filter.
       */
      int getNumParticles() {return mNumParticles;}
      
      /** 
       *  @Function getParticleList.
       *  This is a debugging function for the particlefilter.  It returns a copy
       *  of the particles
       *
       *  @ return all the particles in the particle list.
       */
      ParticleList& getParticleList() {return mParticles;};
      
      /** 
       *  @Function updateParticlePositions.
       *  This function updates the positions of all the particles in the particle filter.
       *  Particles that hit a wall are removed from the list. If the particlefilters population
       *  falls below resamplePercentage.  The list is repopulated with the particles that
       *  have the best sensor reading.
       *
       *  @param odom the odometry reading from the robot.
       *
       *  @ return all the particles in the particle list.
       */
      void updateParticlePositions(Odometry odom);
      
      /** 
       *  @Function updateParticlePositions.
       *  This function compares the laser scan from the robot with each particles laser scan readings
       *  and computes a weight that represents the truthfulness of the laser scan readings.
       *
       *  @param scan the odometry reading from the robot.
       *
       *  @ return void
       */
      void updateParticleSensorData(LaserScan& scan);
            
      /** 
       *  @Function calculatePosition.
       *  This function computes positon of the robot by using a weighted average of the particles.
       *  It is based on the information at http://www.geomidpoint.com/calculation.html 
       *
       *  @ return the robots pose.
       */
      Pose calculatePosition();
      

      void setQrCodePose(Pose qRCodePose);
      
      Pose getQrCodePose() {return mCurrentQrCodePose; };
      
      void QrCodePosePruneParticles();
      
   private:

      /** 
       *  @Function OdomToPose.
       *  This function This is a port of odom_to_pose in utils.py
       *
       *  @ return translation of the odometry into a pose.
       */
      Pose OdomToPose(Odometry& odom);
      
      /** 
       *  @Function wrapTo2Pi.
       *  This function handles the wrapping around of a unit circle angle in radians.
       *
       *  @ return translation of the odometry into a pose.
       */
      float wrapTo2Pi(float theta);
      
      /** 
       *  @Function resample.
       *  This function the repopulates the pool of particles particles when the population falls below
       *  a certain threshold.
       *
       *  @ return None.
       *
       */
      void resample();
      
      /** 
       *  @Function bresenheim.
       *
       *  This is a port from  CODE GENIUS - Drawing Lines with Bresenham's Line Algorithm by Jenn Schiffer
       *  https://www.youtube.com/watch?v=zytBpLlSHms
       *
       *  @param x1pixel y1pixel the starting coordinate in map pixels.
       *  @param x2pixel y2pixel the ending coordinate in map pixels.
       *
       *  @ return true if a wall was found between the start and ending positions.
       *           x2pixel y2pixel  will contain the cordinate of the wall.
       */
      bool bresenheim(uint32_t x1pixel, uint32_t y1pixel, uint32_t& x2pixel, uint32_t& y2pixel);
      
      /** 
       *  @Function bresenheim.
       *
       *  This is a port from corobot-kinetic\corobot_localization\src\bresenheim.py
       *
       *  @param x1pixel y1pixel the starting coordinate in map pixels.
       * 
       *  @ return true if the pixel exceeds the occupancy threshold.
       */
      bool bresenheimCondition(int xpixel, int ypixel);
      
      /** 
       *  @Function getLaserProbability.
       *
       *  This function is based on this is based upon the code corobot-kinetic\corobot_localization\src\kinect_loc.py
       *
       *  @param observedm the robot's observed range in meters.
       *  @param expectedm the particles expected range in meters.
       * 
       *  @ returns the probability that expected range matches the observed range.
       */
      // Handles the error ranges for the laser.
      double getLaserProbability(float observedm, float expectedm);
      
      /** 
       *  @Function getLaserProbability.
       *
       *  This function converts from the cartesian grid to the maps pixel
       *  coordinates.
       *
       *  @param xm ym coordinates in meters.
       * 
       *  @ returns the x and y coordinates in pixels.
       */ 
      void cartesianToGrid(double xm, double ym, uint32_t& xpixel, uint32_t& ypixel);
      
      /** 
       *  @Function getLaserProbability.
       *
       *  This function converts from the maps pixel coodinates to artesian grid
       *  coordinates.
       *
       *  @param xpixel ypixel map coordinates.
       * 
       *  @ returns the x and y coordinates in meters.
       */ 
      void gridToCartesian(uint32_t xpixel, uint32_t ypixel, double& xm, double& ym);
      
      
      /**
        *  class data members
        */
        
      // Holds the last pose from the robot.
      Pose mLastPose;
      
      Pose mCurrentQrCodePose;
      
      // the origin of the grid map matches cartesian coordinates. This does not match the map in 
      // any other part of the system.
      OccupancyGrid mMap;
      
      // The maps height and width in meters.
      float mMapWidthm;
      float mMapHeightm;

      // Holds our particles.
      ParticleList mParticles;
      
      // Number of open spaces in the map.
      int mNumOpenSpaces;
      
      // Number of particles in the map.
      int mNumParticles;
      
      // Number of laser scans we are to process.
      int mNumLaserScans;
      
      // The laser scans we will examine are spread along an evenly distributed arc.
      int* mLaserScanRangeIndex;
      float *mLaserSanRangeAngleRad;
      
      // the minimum and maxium weights that was calculated in the updateParticleSensorData calls
      float mMinWeight;
      float mMaxWeight;
      
      
      int mOrientationRangeDeg;
      // Resample threshold 
      float mResamplePercentage;
      
      // resample threshold in pixels.
      unsigned int mResampleThreshold;
      
      // used to remove pixel from before it crosses the "edge" of the map.
      uint32_t mRoombaRadiusPixels;
      
      // It's a bool jim but not as we know it-- spock.
      bool mInitialized; 
};


#endif