

#ifndef _PARTICLEFILTER_H_
#define _PARTICLEFILTER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <list>
#include "corobot_common/Pose.h"



using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;
using corobot_common::Pose;

typedef struct {
   int particleId;
   Pose pose;
   float weight;
} Particle;



class ParticleFilter
{
   public:
      
      typedef std::list<Particle> ParticleList;
   
      ParticleFilter(OccupancyGrid & map, float resamplePercentage, int orientationRangeDeg);
   
      virtual ~ParticleFilter();
      
      bool initialize (int numParticles, Odometry& startingOdometry);
     
      int getNumParticles() {return mNumParticles;}
      
      ParticleList& getParticleList() {return mParticles;};
      
      void updateParticlePositions(Odometry& odom);
            
   private:
   
      // This is a port of odom_to_pose in utils.py
      Pose OdomToPose(Odometry& odom);
      
      // This function the repopulates the pool of particles particles when the 
      // population drops below a certain threshold.
      void resample();
      
      Pose mLastPose;
   
      OccupancyGrid mMap;
      
      ParticleList mParticles;
      
      int mNumOpenSpaces;
      int mNumParticles;
      
      int mOrientationRangeDeg;
      float mResamplePercentage;
      unsigned int mResampleThreshold;
      
      uint32_t mRoombaRadiusPixels;

   
};


#endif