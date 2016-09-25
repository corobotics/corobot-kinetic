#include "ParticleFilter.h"

#include <ros/ros.h>

ParticleFilter::ParticleFilter(OccupancyGrid & map) :
mNumOpenSpaces(0),
mNumParticles(0)
{
   mMap = map;
   int index = 0;
   int numClosedSpaces = 0;
   
   for(uint32_t y = 0; y < mMap.info.height; ++y)
   {
      
      for(uint32_t x = 0; x < mMap.info.width; ++x)
      {
         index = x + y * mMap.info.width;
         if (mMap.data[index] == 0)
         {
            ++mNumOpenSpaces;
         }
         else
         {
            ++numClosedSpaces;
         }
      }
   }
   
   ROS_INFO("ParticleFilter::ParticleFilter mNumOpenSpaces %d\n", mNumOpenSpaces);
   ROS_INFO("ParticleFilter::ParticleFilter numClosedSpaces %d\n", numClosedSpaces);
   
   
}

ParticleFilter::~ParticleFilter()
{
  // nothing exciting here..
}

bool ParticleFilter::initialize(int numParticles)
{
   Particle particle;
   int index = 0;
   
   if (mNumParticles != numParticles)
   {
      mNumParticles = numParticles;
   }
   
   int spacesPerParticle = mNumOpenSpaces / numParticles;
   
   mParticles.clear();
   
   ROS_INFO("ParticleFilter::ParticleFilter mMap.info.resolution %f\n", mMap.info.resolution);
   
   for(uint32_t y = 0; y < mMap.info.height; ++y)
   {
      for(uint32_t x = 0; x < mMap.info.width; ++x)
      {
         index = x + y * mMap.info.width;
         if (mMap.data[index] == 0)
         {
            if((mNumOpenSpaces % spacesPerParticle) == 0)
            {
               // Store the particles in meters not pixels.
               particle.pose.x = x * mMap.info.resolution;
               particle.pose.y = y * mMap.info.resolution;
               
               // make the orientation random.
               particle.pose.theta = ((index % 360) * M_PI/180);
               
//               cout << "particle.pose.theta " << particle.pose.theta << endl;
               
               mParticles.push_back (particle);
            }
            
            ++mNumOpenSpaces;
         }
      }
   }

   ROS_INFO("ParticleFilter::%s mParticles.size = %lu\n", __func__, mParticles.size());   
}

