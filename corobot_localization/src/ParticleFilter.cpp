#include "ParticleFilter.h"

#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <cstring>

#define INVALIDLASERRANGE 0
int8_t testMap[40][40];





static const int8_t occupancyThreshold = 10;

ParticleFilter::ParticleFilter(OccupancyGrid & map, float resamplePercentage, int orientationRangeDeg, int numLaserScans) :
mNumOpenSpaces(0),
mNumParticles(0),
mNumLaserScans(numLaserScans),
mLaserScanRangeIndex(NULL),
mLaserSanRangeAngleRad(NULL),
mResamplePercentage(resamplePercentage),
mResampleThreshold(0),
mOrientationRangeDeg(orientationRangeDeg),
mInitialized(false)
{
   const float roombaRadiusm = 0.36/2;
   const int NumberOfLaserScanRanges = 640;
   const float scan_angle_min = -0.513185;
   const float scan_angle_increment = 0.001585;
   
   mMap = map;
   int index = 0;
   int numClosedSpaces = 0;
   
   // Calculate the map width;
   mMapWidthm =  mMap.info.width  * mMap.info.resolution;
   mMapHeightm = mMap.info.height * mMap.info.resolution;
   
   for(uint32_t y = 0; y < mMap.info.height; ++y)
   {
      
      for(uint32_t x = 0; x < mMap.info.width; ++x)
      {
         index = x + y * mMap.info.width;
         
         // we're going for each space being 100% open so we can ensure that
         // the particles are placed in a guarenteed open spot.
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
   
   // Do some precomputing for the laser scanner.
   // There are 640 values in the scan so lets precompute which ones we will need to index.
   
   mLaserScanRangeIndex = new int [numLaserScans];
   mLaserSanRangeAngleRad = new float[numLaserScans];
   
   // the reason it's numLaserScans + 1 is so that you get an even distribution of scans across the 1 radian scan arc.
   int laserScanIndexIncrement = NumberOfLaserScanRanges/(numLaserScans + 1);
   
//   ROS_INFO("ParticleFilter::ParticleFilter laserScanIndexIncrement %d\n", laserScanIndexIncrement);
   
   for (int i = 0; i < numLaserScans; ++ i)
   {
      mLaserScanRangeIndex[i] = laserScanIndexIncrement * (i+1);
      mLaserSanRangeAngleRad[i] = scan_angle_min + (scan_angle_increment * mLaserScanRangeIndex[i]);
      
//      ROS_INFO("ParticleFilter::ParticleFilter mLaserScanRangeIndex[%d] %d\n", i, mLaserScanRangeIndex[i]);
//      ROS_INFO("ParticleFilter::ParticleFilter mLaserSanRangeAngleRad[%d] %f\n", i, mLaserSanRangeAngleRad[i]);
   }
   
   // Test code for the bresenheim condition.
   memset(testMap, 0, sizeof(testMap[0][0]) * 20 * 20);
}

ParticleFilter::~ParticleFilter()
{
     // always clean up after yourself.
     delete [] mLaserScanRangeIndex;
     delete [] mLaserSanRangeAngleRad;
}

bool ParticleFilter::initialize(int numParticles)
{
   Particle particle;
   int index = 0;
   
//   ROS_INFO("ParticleFilter::%s called\n", __func__);  
   
   // Distribute the particles evenly.
   if (mNumParticles != numParticles)
   {
      mNumParticles = numParticles;
      
      // Set the resample threshold.
      mResampleThreshold = mNumParticles * mResamplePercentage;
   }
   
   int spacesPerParticle = mNumOpenSpaces / numParticles;
   
   mParticles.clear();
   
//   ROS_INFO("ParticleFilter::%s mMap.info.resolution %f\n", mMap.info.resolution);
   
   for(uint32_t y = 0; y < mMap.info.height; ++y)
   {
      for(uint32_t x = 0; x < mMap.info.width; ++x)
      {
         index = x + y * mMap.info.width;
         
         // Make sure that the space is 100% open.
         if (mMap.data[index] == 0)
         {
            if((mNumOpenSpaces % spacesPerParticle) == 0)
            {
               // Store the particles in meters and pixels.
//               particle.pose.x = x * mMap.info.resolution;
//               particle.pose.y = y * mMap.info.resolution;
               
               gridToCartesian(x, y, particle.pose.x, particle.pose.y);
               
//               ROS_INFO("ParticleFilter::%s 1 particle.pose.x %f particle.pose.y %f x %u y %u\n", __func__, particle.pose.x, particle.pose.y, x, y);
               
               particle.poseXpixels = x;
               particle.poseYpixels = y;
               
               // make the orientation random.
               // theta and start theta should be the same.
               particle.pose.theta = ((index % 360) * M_PI/180);
               
               particle.pose.theta = wrapTo2Pi(particle.pose.theta);
               
               // The weight is always 1 until we know it's false.
                particle.weight = 1;
               
               mParticles.push_back (particle);
            }
            
            ++mNumOpenSpaces;
         }
      }
   }
   
//   ROS_INFO("ParticleFilter::%s done\n", __func__);  
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

   
//   ROS_INFO("ParticleFilter::%s called\n", __func__);
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
      //ROS_INFO("ParticleFilter::%s called normalized it->weight %f\n", __func__, it->weight);
      
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

            particle.pose.theta = wrapTo2Pi(particle.pose.theta);
            
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

   ROS_INFO("ParticleFilter::%s newParticles %lu mNumParticles = %d\n", __func__, newParticles.size(), mNumParticles);  
   
/*
   for(ParticleFilter::ParticleList::iterator it = mParticles.begin(); it != mParticles.end(); ++it)
   {
      ROS_INFO("ParticleFilter::%s mParticles x = %f y = %f weight = %f theta = %f\n", __func__, it->pose.x, it->pose.y, it->weight, it->pose.theta);  
   }
   
   for(ParticleFilter::ParticleList::iterator it = newParticles.begin(); it != newParticles.end(); ++it)
   {
      ROS_INFO("ParticleFilter::%s newParticles x = %f y = %f weight = %f theta = %f \n", __func__, it->pose.x, it->pose.y, it->weight, it->pose.theta); 
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
   uint32_t index = 0;
//   ROS_INFO("ParticleFilter::%s called\n", __func__);  
   
   currentPose = OdomToPose(odom);
   
   // if we just initialzied then update the current pose otherwise update the particles;
   if(mInitialized == false)
   {
      
      // Check to see if we need to resample;
      if (mParticles.size() < mResampleThreshold)
      {
         resample();
      }
      
      
//      ROS_INFO("ParticleFilter::%s  mInitialized = false\n", __func__);
   //   ROS_INFO("ParticleFilter::%s currentPose.x %f  currentPose.y %f currentPose.theta %f\n", __func__, currentPose.x, currentPose.y, currentPose.theta);
   //   ROS_INFO("ParticleFilter::%s mLastPose.x %f  mLastPose.y %f mLastPose.theta %f\n", __func__, mLastPose.x, mLastPose.y, mLastPose.theta);
      
      diffPose.x = currentPose.x - mLastPose.x;
      diffPose.y = currentPose.y - mLastPose.y;
      diffPose.theta = currentPose.theta - mLastPose.theta;
      
      diffPose.theta = wrapTo2Pi(diffPose.theta);
      
   //   ROS_INFO("ParticleFilter::%s diffPose.x %f  diffPose.y %f\n", __func__, diffPose.x, diffPose.y);
      
      if(diffPose.x != 0 || diffPose.y != 0)
      {
         alpha = atan2 (diffPose.y, diffPose.x) + mLastPose.theta;
      }
      
   //   ROS_INFO("ParticleFilter::%s alpha %f\n", __func__, alpha);
      
      // Calculate rho!
      rho = sqrt((pow(diffPose.x, 2) + pow(diffPose.y, 2)));
      
   //   ROS_INFO("ParticleFilter::%s rho %f\n", __func__, rho);
      
      // Update the particle positions
      ParticleFilter::ParticleList::iterator it = mParticles.begin();
      while ( it != mParticles.end())
      {

   //      ROS_INFO("ParticleFilter::%s it->pose.theta %f\n", __func__, it->pose.theta);
         
         // Now that we're facing the "correct" direction up date the x and y coordinates.
         it->pose.x = it->pose.x + rho * cos( it->pose.theta + alpha);
         it->pose.y = it->pose.y + rho * sin( it->pose.theta + alpha);
         it->pose.theta += diffPose.theta;
         
         it->pose.theta = wrapTo2Pi(it->pose.theta);
        
   //      ROS_INFO("ParticleFilter::%s it->pose.x %f, it->pose.y %f, it->pose.theta %f\n", __func__, it->pose.x, it->pose.y , it->pose.theta);
         
         // check to see if we crashed into a wall. If we did the particle is false so
         // kill it.
         cartesianToGrid(it->pose.x, it->pose.y, it->poseXpixels,it->poseYpixels);
         index = it->poseXpixels + (it->poseYpixels * mMap.info.width);

         // Check to make sure that the particle has not gone off the map
         // TODO we need to do the same calculations in X and Y land to see if we've crashed into a wall.
         if( it->poseXpixels < mRoombaRadiusPixels || it->poseXpixels > (mMap.info.width - mRoombaRadiusPixels) ||
             it->poseYpixels < mRoombaRadiusPixels || it->poseYpixels > (mMap.info.height - mRoombaRadiusPixels))
         {
             ROS_INFO("ParticleFilter::%s erasing it->pose.theta %f (off the map)\n", __func__, it->pose.theta);
             it = mParticles.erase(it);
         }
         else if (mMap.data[index] != 0)
         {
             // Check to see if we crashed into a wall.
             uint8_t temp = mMap.data[index];
             ROS_INFO("ParticleFilter::%s erasing it->pose.theta %f mapdata = %d\n", __func__, it->pose.theta, temp);
             it = mParticles.erase(it);
         }
         else
         {
            ++it;
         }
   //         ROS_INFO("ParticleFilter::%s it->pose x = %f, y = %f, theta = %f\n", __func__, it->pose.x, it->pose.y it->pose.theta);
      }
   }
   else
   {
      ROS_INFO("ParticleFilter::%s  mInitialized = true\n", __func__);
      ROS_INFO("ParticleFilter::%s it->pose x = %f, y = %f, theta = %f\n", __func__, currentPose.x, currentPose.y, currentPose.theta);
      mInitialized = false;
   }
   
   mLastPose = currentPose;
 //  ROS_INFO("ParticleFilter::%s done\n", __func__);  
}

// This function is based upon the work in corobot_localization\src\kinect_loc.py
void ParticleFilter::updateParticleSensorData(LaserScan& scan)
{
   // some interesting data about the laser
//   ROS_INFO("ParticleFilter::%s  called\n", __func__);
//        ROS_INFO("ParticleFilter::%s  angle_min %f\n", __func__, scan.angle_min);
//        ROS_INFO("ParticleFilter::%s  angle_max called %f\n", __func__, scan.angle_max);
//        ROS_INFO("ParticleFilter::%s  angle_increment %f\n", __func__, scan.angle_increment);
//        ROS_INFO("ParticleFilter::%s  range_min %f\n", __func__, scan.range_min);
//        ROS_INFO("ParticleFilter::%s  range_max %f\n", __func__, scan.range_max);
   
   // The particle positions need to be updated at least once before we do the scanner information.
//   if(mInitialized == false)
   {
      ROS_INFO("ParticleFilter::%s  mInitialized = false  Process laser data.\n", __func__);

      // Check to see if the laser data matches the particle. 
      ParticleFilter::ParticleList::iterator it = mParticles.begin();
 
      float scanThetaRad = 0;
      float laserEndXm = 0;
      float laserEndYm = 0;
      int laserEndXpixel = 0;
      int laserEndYpixel = 0;
      float particleLaserRangem = 0;
      
      bool occupied = false;
      
      while ( it != mParticles.end())
      {
         occupied = false;
         
         ROS_INFO("ParticleFilter::%s it->pose x = %f, y = %f\n", __func__, it->pose.x, it->pose.y);
         
         // examine each scan and see if it intersects.
         for(int i = 0; i < mNumLaserScans; ++i)
         {
            particleLaserRangem = 0;
                        
            scanThetaRad = it->pose.theta + mLaserSanRangeAngleRad[i];
            scanThetaRad = wrapTo2Pi(scanThetaRad);
            
            ROS_INFO("ParticleFilter::%s it->pose.theta %f\n", __func__, it->pose.theta); 
            ROS_INFO("ParticleFilter::%s it->scanThetaRad %f\n", __func__, scanThetaRad); 
            
            // Get the end point of the laser reading in meters.
            laserEndXm = it->pose.x  + mLaserRangeMaxm * cos(scanThetaRad);
            laserEndYm = it->pose.y  +  mLaserRangeMaxm * sin(scanThetaRad);
            
            // Make sure the laser stays in left and right sides of the map. 
            
            // If you change x then you need to change y to match the value of the angle
            if(laserEndXm < 0)
            {
               laserEndXm = 0; 
               
               // Only update y if it affects the y coordinate.
               if(scanThetaRad !=M_PI && scanThetaRad != -M_PI)
               {
                  // y = x * tan (theta)
                  laserEndYm = (it->pose.y + it->pose.x * tan(scanThetaRad));
               }
            }
            else if (laserEndXm >= mMapWidthm)
            {  
               laserEndXm = mMapWidthm;
               
               // Only update y if it affects the y coordinate.
               if(scanThetaRad !=M_PI && scanThetaRad != -M_PI)
               {
                  // y = xdiff * tan (theta)
                  laserEndYm = (it->pose.y + (mMapWidthm - it->pose.x) * tan(scanThetaRad));
               }
            }
            
            // Check to make sure we don't run off the top and bottom of the map.
            if(laserEndYm < 0)
            {
               laserEndYm = 0;
               
               // Only update x if it affects the x coordinate.
               if(scanThetaRad !=M_PI && scanThetaRad != -M_PI)
               {
                  // x = y / tan (theta)
                  laserEndXm = (it->pose.x + it->pose.y / tan(scanThetaRad));
               }
            }
            else if (laserEndYm >= mMapHeightm)
            {
               laserEndYm = mMapHeightm;
               
               // Only update x if it affects the x coordinate.
               if(scanThetaRad !=M_PI && scanThetaRad != -M_PI)
               {
                  // x = ydiff / tan (theta)
                  laserEndXm = (it->pose.x + (mMapHeightm - it->pose.y)/tan(scanThetaRad));
               }
            }
            
            // Now we can do the ray tracing.
            laserEndXpixel = laserEndXm / mMap.info.resolution;
            laserEndYpixel = laserEndYm / mMap.info.resolution;
            
            occupied = bresenheim(it->poseXpixels, it->poseYpixels,  laserEndXpixel, laserEndYpixel);
            
            ROS_INFO("ParticleFilter::%s occupied %d\n", __func__, occupied);
            // Hey we got something!!  Calculate the range to the laser.
            // were calculating everything in meters;
            if(occupied)
            {
               ROS_INFO("ParticleFilter::%s occupied it->poseXpixels %f it->poseYpixels %f laserEndXpixel %d laserEndYpixel %d\n", __func__, it->pose.x, it->pose.y, laserEndXpixel, laserEndYpixel);
               
               particleLaserRangem = sqrt(pow((it->pose.x - laserEndXpixel * mMap.info.resolution), 2) + pow((it->pose.y - laserEndYpixel * mMap.info.resolution), 2));
               ROS_INFO("ParticleFilter::%s occupied particleLaserRangem %f\n", __func__, particleLaserRangem);
            }
            
           
         }
         
         ++it;
      }
   }
   
//   ROS_INFO("ParticleFilter::%s done\n", __func__);  
}

float ParticleFilter::wrapTo2Pi(float theta)
{
   if (theta > M_PI)
   {
      // Counterclockwise
      theta -= 2 * M_PI;
     
   }
   else if (theta < -M_PI)
   {
      // Clockwise
      theta += 2 * M_PI;  
   } 
   
   return theta;
}


bool ParticleFilter::bresenheim(int x1pixel, int y1pixel, int& x2pixel, int& y2pixel)
{
   ROS_INFO("ParticleFilter::%s  start x1pixel %d y1pixel %d x2pixel %d y2pixel %d\n", __func__, x1pixel, y1pixel, x2pixel, y2pixel);
   bool occupied = false;
   
   if(x1pixel == x2pixel && y1pixel == y2pixel)
   {
      if(bresenheimCondition(x1pixel, y1pixel))
      {
         occupied = true;
      }
   }
   else
   {
      float dxpixel = x2pixel - x1pixel;
      float dypixel = y2pixel - y1pixel;
      
      int incX = (dxpixel < 0) ? -1 : 1;
      int incY = (dypixel < 0) ? -1 : 1;
      
      
      float m = 0;
      float b = 0;
      
//      ROS_INFO("ParticleFilter::%s  dxpixel %f\n", __func__, dxpixel);
//      ROS_INFO("ParticleFilter::%s  dypixel %f\n", __func__, dypixel);
      
      if(abs(dypixel) < abs(dxpixel))
      {
         int tempXpixel = x1pixel;
         m = dypixel/dxpixel;
         ROS_INFO("ParticleFilter::%s  m %f\n", __func__, m);
         b = y1pixel - m * x1pixel;
         ROS_INFO("ParticleFilter::%s  b %f\n", __func__, b);
         
         while(tempXpixel != x2pixel && occupied == false)
         {
            occupied = bresenheimCondition(tempXpixel, (m * tempXpixel +  b));
            if( occupied == false)
            {
               tempXpixel += incX;
            }
         }
         
         // this handles the corner case where tempXpixel == x2pixel
         if( occupied == false)
         {
            occupied = bresenheimCondition(tempXpixel, (m * tempXpixel +  b));
         }
         
         // We hit a wall. update the x2pixel and y2pixel values.
         if(occupied == true)
         {
            x2pixel = tempXpixel;
            y2pixel = (m * tempXpixel +  b);
         }
         
      }
      else
      {
         int tempYpixel = y1pixel;
         m = dxpixel/dypixel;
         ROS_INFO("ParticleFilter::%s  m %f\n", __func__, m);
         b = x1pixel - m * y1pixel;
         ROS_INFO("ParticleFilter::%s  b %f\n", __func__, b);
         
         while(tempYpixel != y2pixel && occupied == false)
         {
            occupied = bresenheimCondition((m * tempYpixel +  b), tempYpixel);

            if( occupied == false)
            {
               tempYpixel += incY;
            }
         }
         
         // this handles the corner case where tempYpixel == y2pixel
         if( occupied == false)
         {
            occupied = bresenheimCondition((m * tempYpixel +  b), tempYpixel);
         }
         
          // We hit a wall.  Calculate the range from X1, Y1 to the wall.
         if(occupied == true)
         {
            x2pixel = (m * tempYpixel +  b);
            y2pixel = tempYpixel;
         }
      }
   }
   
   ROS_INFO("ParticleFilter::%s  end x2pixel %d y2pixel %d occupied %d\n", __func__, x2pixel, y2pixel, occupied);
   return occupied;
}

bool ParticleFilter::bresenheimCondition(int xpixel, int ypixel)
{
   bool occupied = false;
     
   int index = xpixel + ypixel * mMap.info.width;

   
  
//    if(testMap[xpixel][ypixel]  > occupancyThreshold)
   if (mMap.data[index] > occupancyThreshold)
   {
      occupied = true;
   }
   
//   ROS_INFO("ParticleFilter::%s  xpixel %d ypixel %d occupied %d\n", __func__, xpixel, ypixel, occupied);
   
   return occupied;
}

float ParticleFilter::getLaserProbability(float observedm, float expectedm)
{
   
    // probability on y axis
    float p1 = 0.4; // obstacle closer than expected
    float p2 = 0.6; // obstacle near expected
    float p3 = 0.2; // obstacle farther than expected

    // differences on x axis
    float d1 = 0.8;
    float d2 = 0.95;
    float d3 = 1.05;
    float d4 = 1.2;
    
    float probability = 1.0;
    
    // this will help in the case were the particle and the laser are NANs.

    // If the laser data is valid then process it. This also handles the case were
    // both the laser and the particle are outside of the valid ranges by returning 1.0
    if(observedm >= mLaserRangeMinm  || observedm <= mLaserRangeMaxm)
    {
      float ratio = observedm / expectedm;
      
       if(ratio <= d1)
       {
           probability = p1;
       }
       else if (ratio > d1 && ratio <= d2)
       {
           probability = (((p2 - p1)*(ratio - d1))/(d2 - d1)) + p1;
       }
       else if (ratio > d2 && ratio <= d3)
       {
           probability = p2;
       }
       else if (ratio > d3 and ratio <= d4)
       {
           probability =(((p3 - p2)*(ratio - d3))/(d4 - d3)) + p2;
       }
       else
       {
           return p3;
       }
    }
   
   return probability;
}

void ParticleFilter::cartesianToGrid(double x, double y, uint32_t& xpixel, uint32_t& ypixel)
 {
    xpixel = x / mMap.info.resolution;
    ypixel = mMap.info.height - ( y / mMap.info.resolution);
 }
 
void ParticleFilter::gridToCartesian(uint32_t xpixel, uint32_t ypixel, double& x, double& y)
{
    x = xpixel * mMap.info.resolution;
    y = (mMap.info.height - ypixel) * mMap.info.resolution;
    
}
   