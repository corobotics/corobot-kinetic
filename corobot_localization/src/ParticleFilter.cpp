#include "ParticleFilter.h"

#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <vector>

ParticleFilter::ParticleFilter(OccupancyGrid & map, float resamplePercentage, int orientationRangeDeg) :
mNumOpenSpaces(0),
mNumParticles(0),
mResamplePercentage(resamplePercentage),
mResampleThreshold(0),
mOrientationRangeDeg(orientationRangeDeg),
mInitialized(false)
{
   const float roombaRadiusm = 0.36/2;
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
   
   mRoombaRadiusPixels = (roombaRadiusm / mMap.info.resolution);
   
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
   
   // Distribute the particles evenly.
   if (mNumParticles != numParticles)
   {
      mNumParticles = numParticles;
      
      // Set the resample threshold.
      mResampleThreshold = mNumParticles * mResamplePercentage;
   }
   
   int spacesPerParticle = mNumOpenSpaces / numParticles;
   
   mParticles.clear();
   
//   ROS_INFO("ParticleFilter::ParticleFilter mMap.info.resolution %f\n", mMap.info.resolution);
   
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
               particle.pose.theta = ((index % 360) * M_PI/180);
               
               // We're using a circle which is a 4 quadrant inverse tangent.
               // we only have to check the zero crossing in the counter clockwise direction.
               if(particle.pose.theta > M_PI)
               {
                  particle.pose.theta -= 2 * M_PI;
               }
               
               // The weight is always 1 until we know it's false.
                particle.weight = 1;
               
               mParticles.push_back (particle);
            }
            
            ++mNumOpenSpaces;
         }
      }
   }
   
   mInitialized = true;

   ROS_INFO("ParticleFilter::%s mParticles.size = %lu\n", __func__, mParticles.size());  

}

Pose ParticleFilter::OdomToPose(Odometry& odom)
{
   Pose pose;
   double qz = 0;
   double qw = 0;
   
   pose.x = odom.pose.pose.position.x;
   pose.y = odom.pose.pose.position.y;
   
   // Get the Robot orientation.
   qz = odom.pose.pose.orientation.z;
   qw = odom.pose.pose.orientation.w;
   
   pose.theta = atan2(2 * qw * qz, 1 - 2 * pow(qz, 2));
   
   return pose;
}

void ParticleFilter::resample()
{
   std::vector <float> cumsums;
   std::vector <float> randomNums;
   ParticleList newParticles;
   Particle particle;
   int i = 0;
   int j = 0;
   float cumWeight = 0;
   bool changeOrientation = false;
   double temp = 0;

   
   ROS_INFO("PFLocalizationNode::%s called\n", __func__);
   // first write the test code since we need it.
   // Initialize the weights to values [0..1]
/*   
   for(ParticleFilter::ParticleList::iterator it = mParticles.begin(); it != mParticles.end(); ++it)
   {
      it->weight = ((float)(rand() % 1000) / 1000);
   }
*/
   // end of test code.

   // Just calculate the cumulative weight.
   for(ParticleFilter::ParticleList::iterator it = mParticles.begin(); it != mParticles.end(); ++it)
   {
//         it->weight = ((it->weight - minWeight)/weightRange);
      //ROS_INFO("PFLocalizationNode::%s called normalized it->weight %f\n", __func__, it->weight);
      
      // Now that things are normalized lets generate our cumsum vector;
      cumWeight += it->weight;
      cumsums.push_back(cumWeight);
   }
   
   for(int i = 0; i < mNumParticles + 1; ++i)
   {
      randomNums.push_back(((float)(rand() % 1000) / 1000));
   }
   
   
   std::sort (randomNums.begin(), randomNums.end());

   ParticleFilter::ParticleList::iterator it = mParticles.begin();
   while (i < mNumParticles)
   {
      if(randomNums[i] < (cumsums[j] / cumWeight))
      {
         if(changeOrientation == true)
         {
            // make a deep copy of the particle and changed the orientation.
            particle = (*it);
            
            temp = rand() % mOrientationRangeDeg;
            temp -= (mOrientationRangeDeg / 2);
            temp *= (M_PI/180);
            
            
            particle.pose.theta += temp;

            if (particle.pose.theta > M_PI)
            {
               // Counterclockwise
               particle.pose.theta -= 2 * M_PI;
              
            }
            else if (particle.pose.theta < -M_PI)
            {
               // Clockwise
               particle.pose.theta += 2 * M_PI;  
            }            
            
            newParticles.push_back(particle);
         }
         else
         {
            newParticles.push_back(*it);
         }
         ++i;
         // The first copy is an exact copy of the origional.  The other
         // particles will have changed orientations.
         changeOrientation = true;
      }
      else
      {
         ++j;
         if(it != mParticles.end())
         {
            ++it;
         }
         changeOrientation = false;
      }
   }
   
      // Debug code

   ROS_INFO("PFLocalizationNode::%s newParticles %lu mNumParticles = %d\n", __func__, newParticles.size(), mNumParticles);  
   
/*
   for(ParticleFilter::ParticleList::iterator it = mParticles.begin(); it != mParticles.end(); ++it)
   {
      ROS_INFO("PFLocalizationNode::%s mParticles x = %f y = %f weight = %f theta = %f\n", __func__, it->pose.x, it->pose.y, it->weight, it->pose.theta);  
   }
   
   for(ParticleFilter::ParticleList::iterator it = newParticles.begin(); it != newParticles.end(); ++it)
   {
      ROS_INFO("PFLocalizationNode::%s newParticles x = %f y = %f weight = %f theta = %f \n", __func__, it->pose.x, it->pose.y, it->weight, it->pose.theta); 
   }   
*/
   // This is our new list of particles;
   mParticles = newParticles;
}

void ParticleFilter::updateParticlePositions(Odometry& odom)
{
   Pose currentPose;
   Pose diffPose;
   double rho = 0;
   double alpha = 0;
   uint32_t mapPosx = 0;
   uint32_t mapPosy = 0;
   uint32_t index = 0;
   
   currentPose = OdomToPose(odom);
   
   // if we just initialzied then update the current pose otherwise update the particles;
   if(mInitialized == false)
   {
      
      ROS_INFO("PFLocalizationNode::%s  mInitialized = false\n", __func__);
   //   ROS_INFO("PFLocalizationNode::%s currentPose.x %f  currentPose.y %f currentPose.theta %f\n", __func__, currentPose.x, currentPose.y, currentPose.theta);
   //   ROS_INFO("PFLocalizationNode::%s mLastPose.x %f  mLastPose.y %f mLastPose.theta %f\n", __func__, mLastPose.x, mLastPose.y, mLastPose.theta);
      
      diffPose.x = currentPose.x - mLastPose.x;
      diffPose.y = currentPose.y - mLastPose.y;
      diffPose.theta = currentPose.theta - mLastPose.theta;
      
      if (diffPose.theta > M_PI)
      {
         // Counterclockwise
         diffPose.theta -= 2 * M_PI;
        
      }
      else if (diffPose.theta < -M_PI)
      {
         // Clockwise
         diffPose.theta += 2 * M_PI;  
      }
      
   //   ROS_INFO("PFLocalizationNode::%s diffPose.x %f  diffPose.y %f\n", __func__, diffPose.x, diffPose.y);
      
      if(diffPose.x != 0 || diffPose.y != 0)
      {
         alpha = atan2 (diffPose.y, diffPose.x) + mLastPose.theta;
      }
      
   //   ROS_INFO("PFLocalizationNode::%s alpha %f\n", __func__, alpha);
      
      // Calculate rho!
      rho = sqrt((pow(diffPose.x, 2) + pow(diffPose.y, 2)));
      
   //   ROS_INFO("PFLocalizationNode::%s rho %f\n", __func__, rho);
      
      // Update the particle positions
      ParticleFilter::ParticleList::iterator it = mParticles.begin();
      while ( it != mParticles.end())
      {

   //      ROS_INFO("PFLocalizationNode::%s it->pose.theta %f\n", __func__, it->pose.theta);
         
         // Now that we're facing the "correct" direction up date the x and y coordinates.
         it->pose.x = it->pose.x + rho * cos( it->pose.theta + alpha);
         it->pose.y = it->pose.y + rho * sin( it->pose.theta + alpha);
         it->pose.theta += diffPose.theta;
         
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
         
   //      ROS_INFO("PFLocalizationNode::%s it->pose.x %f, it->pose.y %f, it->pose.theta %f\n", __func__, it->pose.x, it->pose.y , it->pose.theta);
         
         // check to see if we crashed into a wall. If we did the particle is false so
         // kill it.
         
         mapPosx = it->pose.x / mMap.info.resolution;
         mapPosy = it->pose.y / mMap.info.resolution;
         index = mapPosx + (mapPosy * mMap.info.width);

         // Check to make sure that the particle has not gone off the map
         if( mapPosx < mRoombaRadiusPixels || mapPosx > (mMap.info.width - mRoombaRadiusPixels) ||
             mapPosy < mRoombaRadiusPixels || mapPosy > (mMap.info.height - mRoombaRadiusPixels))
         {
   //          ROS_INFO("PFLocalizationNode::%s erasing it->pose.theta %f (off the map)\n", __func__, it->pose.theta);
             it = mParticles.erase(it);
         }
         else if (mMap.data[index] != 0)
         {
             uint8_t temp = mMap.data[index];
   //          ROS_INFO("PFLocalizationNode::%s erasing it->pose.theta %f mapdata = %d\n", __func__, it->pose.theta, temp);
             it = mParticles.erase(it);
         }
         else
         {
            ++it;
         }
   //         ROS_INFO("PFLocalizationNode::%s it->pose x = %f, y = %f, theta = %f\n", __func__, it->pose.x, it->pose.y it->pose.theta);
      }
      
      // Check to see if we need to resample;
      if (mParticles.size() < mResampleThreshold)
      {
         resample();
      }
   }
   else
   {
      ROS_INFO("PFLocalizationNode::%s  mInitialized = true\n", __func__);
      ROS_INFO("PFLocalizationNode::%s it->pose x = %f, y = %f, theta = %f\n", __func__, currentPose.x, currentPose.y, currentPose.theta);
      mInitialized = false;
   }
   
   mLastPose = currentPose;
}