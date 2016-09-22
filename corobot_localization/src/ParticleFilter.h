

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
} Particle;



class ParticleFilter
{
   public:
      
      typedef std::list<Particle> ParticleList;
   
      ParticleFilter(OccupancyGrid & map);
   
      virtual ~ParticleFilter();
      
      bool initialize (int numParticles);
     
      int getNumParticles() {return mNumParticles;}
      
      ParticleList getParticleList() {return mParticles;};
            
   private:
   
      OccupancyGrid mMap;
      
      ParticleList mParticles;
      
      int mNumOpenSpaces;
      int mNumParticles;
   
};


#endif