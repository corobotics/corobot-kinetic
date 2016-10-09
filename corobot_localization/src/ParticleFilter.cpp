#include "ParticleFilter.h"

#include <ros/ros.h>
#include <math.h>

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

bool ParticleFilter::initialize(int numParticles, Odometry& startingOdometry)
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
               // theta and start theta should be the same.
               particle.startTheta = ((index % 360) * M_PI/180);
               particle.pose.theta = particle.startTheta;

               
               // We're using a circle which is a 4 quadrant inverse tangent.
               // we only have to check the zero crossing in the counter clockwise direction.
               if(particle.startTheta > M_PI)
               {
                  particle.startTheta -= 2 * M_PI;
               }
               
               mParticles.push_back (particle);
            }
            
            ++mNumOpenSpaces;
         }
      }
   }

   ROS_INFO("ParticleFilter::%s mParticles.size = %lu\n", __func__, mParticles.size());  

   mLastPose = OdomToPose(startingOdometry);
}

Pose ParticleFilter::OdomToPose(Odometry& odom)
{
   Pose pose;
   
   pose.x = odom.pose.pose.position.x;
   pose.y = odom.pose.pose.position.y;
   
   return pose;
}

void ParticleFilter::updateParticlePositions(Odometry& odom)
{
   Pose currentPose;
   Pose diffPose;
   double rho = 0;
   double diffTheta = 0;
   uint32_t mapPosx = 0;
   uint32_t mapPosy = 0;
   uint32_t index = 0;
   
   currentPose = OdomToPose(odom);
   
   ROS_INFO("PFLocalizationNode::%s currentPose.x %f  currentPose.y %f\n", __func__, currentPose.x, currentPose.y);
   ROS_INFO("PFLocalizationNode::%s mLastPose.x %f  mLastPose.y %f\n", __func__, mLastPose.x, mLastPose.y);
   
   diffPose.x = currentPose.x - mLastPose.x;
   diffPose.y = currentPose.y - mLastPose.y;
   ROS_INFO("PFLocalizationNode::%s diffPose.x %f  diffPose.y %f\n", __func__, diffPose.x, diffPose.y);
   
   if(diffPose.x != 0 || diffPose.y != 0)
   {
      diffTheta = atan2 (diffPose.y, diffPose.x);
   }
   
   ROS_INFO("PFLocalizationNode::%s difftheta %f\n", __func__, diffTheta);
   
   // Calculate rho!
   rho = sqrt((pow(diffPose.x, 2) + pow(diffPose.y, 2)));
   
   ROS_INFO("PFLocalizationNode::%s rho %f\n", __func__, rho);
   
   // Update the particle positions
   ParticleFilter::ParticleList::iterator it = mParticles.begin();
   while ( it != mParticles.end())
   {
      // First update the orientation which is based on the starting orientation of the particle;
      it->pose.theta = diffTheta + it->startTheta;
      
      // We're using a circle which is a 4 quadrant inverse tangent.
      // Fix up crossing the PI/-PI boundary 
      if (it->pose.theta > M_PI)
      {
         // Counterclockwise
         it->pose.theta -= 2 * M_PI;
        
      }
      else if (it->pose.theta < -M_PI)
      {
         // Clockwise
         it->pose.theta += 2 * M_PI;  
      }
      
      // Now that we're facing the "correct" direction up date the x and y coordinates.
      it->pose.x = it->pose.x + rho * cos( it->pose.theta);
      it->pose.y = it->pose.y + rho * sin( it->pose.theta);
      
            // check to see if we crashed into a wall. If we did the particle is false so
      // kill it.
      
      mapPosx = it->pose.x / mMap.info.resolution;
      mapPosy = it->pose.y / mMap.info.resolution;
      
      index = mapPosx + (mapPosy * mMap.info.width);
      
      if (mMap.data[index] != 0)
      {
          uint8_t temp = mMap.data[index];
          ROS_INFO("PFLocalizationNode::%s erasing it->pose.theta %f mapdata = %d\n", __func__, it->pose.theta, temp);
          it = mParticles.erase(it);
      }
      else
      {
         ++it;
      }
//         ROS_INFO("PFLocalizationNode::%s it->pose x = %f, y = %f, theta = %f\n", __func__, it->pose.x, it->pose.y it->pose.theta);
   }
   
   // Now that we've updated the postions check to see if any of the particles crashed into a wall and remove them from the list.
   
   mLastPose = currentPose;
}