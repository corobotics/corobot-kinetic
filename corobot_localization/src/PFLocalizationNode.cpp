//*************************************************************************************************
//  This file provides the interface to the corobot and maintains the state of the particlefiler.
//
//*************************************************************************************************


#include <ros/ros.h>
#include <ros/types.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <mutex>

#include "corobot_common/GetCoMap.h"
#include "corobot_common/Pose.h"
#include "corobot_common/PoseArray.h"

#include "ParticleFilter.h"

using namespace ros;
using geometry_msgs::Point;
using nav_msgs::Odometry;
using corobot_common::Pose;
using corobot_common::PoseArray;
using sensor_msgs::LaserScan;

ros::Publisher particlePublisher;
ros::Publisher pfPosePublisher;

ParticleFilter* particleFilter = NULL;

void odomCallback(Odometry odom);
void goalCallback(Point goal);
void qrcodePoseCallback(Pose qrCodePose);
void scanCallback(LaserScan scan);

// Processing and initializing states for the particle filter.
typedef enum {
  STOPPED,
  IDLE,
  INITIALIZING,
  INITIALIZED,
  RUNNING,
  PROCESSODOM,
  PROCESSINGODOM,
  PROCESSSCAN,
  PROCESSINGSCAN,
  STATEMAX
} ParticleFilterState;

// Initial state. 
ParticleFilterState particleFilterState = STOPPED;

static unsigned odomCounter = 0;
static unsigned scanCounter = 0;

Odometry testVector[8];
extern int8_t testMap[40][40];

std::mutex PFLocMutex;

static bool qRcodePoseRecieved = false;
static bool reinitializeParticleFilter = false;
static int numberofParticles = 1000;
   
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   
   ros::init(argc, argv, "corobot_localization_node");
   
   
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */


   ros::Subscriber goalSub = n.subscribe("goals_nav", 1, goalCallback);
   
   
   // tscan and todom are throttled scan and odom messages.
   ros::Subscriber odomSub = n.subscribe("todom", 10, odomCallback);
   ros::Subscriber scanSub = n.subscribe("tscan", 10, scanCallback);
   ros::Subscriber qrcodeSub = n.subscribe("tqrcode_pose", 5, qrcodePoseCallback);

   
   // Publishes the Particle filters estimated location.
   pfPosePublisher = n.advertise<corobot_common::Pose>("particleFilterPose", 1);

   // Debug Publishes the Particle filters particles.
   particlePublisher = n.advertise<corobot_common::PoseArray>("particleList", 1);
   
   // Get the map for the particle filter.
   ros::service::waitForService("get_map");
   corobot_common::GetCoMap coMap;
   
   if (ros::service::call("get_map", coMap))
   {

      particleFilter = new ParticleFilter(coMap.response.map, 0.7, 120, 10);
      
      // we're up and running so transition to IDLE.
      particleFilterState = IDLE;
      
      // Print out some debug so that we know that we started up.
      ROS_INFO("PFLocalizationNode::%s started waiting for initialization\n", __func__);    
   }
   else
   {
      ROS_INFO("No map found. Particle filter failed to start!");
   }
   
   ros::spin();
}

void goalCallback(Point goal)
{
   ROS_INFO("PFLocalizationNode::%s called x = %f, y = %f\n",  __func__, goal.x, goal.y);
      // If we recieved a new goal reinitialze the particle filter.
   if(particleFilterState == IDLE || particleFilterState == RUNNING)
   {
      particleFilterState = INITIALIZING;
      
      PFLocMutex.lock();

      particleFilter->initialize(numberofParticles);
      
      PFLocMutex.unlock();      
            
      ROS_INFO("PFLocalizationNode::%s initialization completed\n", __func__);
 
      particleFilterState = PROCESSODOM;

   } 
   else
   {
      ROS_INFO("odomCallback ERROR: recieved a new goal when in state %d\n", particleFilterState);
   }
}
//*************************************************************************************************
//  Function: Odom Callback
//           This function updates the particle filter with the latest odom message and then
//           publishes the robot pose from the particle filter.
//
//  Parameters odom an odometry reading from the robot.
//
//  returns: none.   
//
//*************************************************************************************************
void odomCallback(Odometry odom)
{
      
   double x_m = odom.pose.pose.position.x;
   double y_m = odom.pose.pose.position.y;
   double z_m = 0;
  
//   ROS_INFO("PFLocalizationNode::%s odomCounter = %u\n", __func__, odomCounter);
 
   
   if (particleFilterState == PROCESSODOM)
   {
      // change the state to  PROCESSINGODOM in case this function is not thread safe. 
      particleFilterState = PROCESSINGODOM;
      PFLocMutex.lock();
     
      ROS_INFO("PFLocalizationNode::%s PROCESSINGODOM x = %f, y = %f\n", __func__, x_m, y_m); 
      
      PoseArray particles; 
      
      // Hack for significant change in the QR codes.
      if(reinitializeParticleFilter == true)
      {
         particleFilter->initialize(numberofParticles);
         reinitializeParticleFilter = false;
      }
      
      particleFilter->updateParticlePositions(odom);
 
      // calculate the position of the robot and sent it to the UI.
      Pose location;
      location = particleFilter->calculatePosition();
      
      pfPosePublisher.publish(location);
/*      
      // Debug code sends the particles to the corobot map
      ParticleFilter::ParticleList& results = particleFilter->getParticleList();
      
      for (ParticleFilter::ParticleList::iterator it=results.begin(); it != results.end(); ++it)
      {
//         ROS_INFO("PFLocalizationNode::%s it->pose x = %f, y = %f\n", __func__, it->pose.x, it->pose.y);
         particles.poses.push_back((it->pose));
      }
      
       ROS_INFO("PFLocalizationNode::%s particles = %lu\n", __func__, particles.poses.size());
      
      particlePublisher.publish(particles);
*/
      // Make sure we process the corresponding scan
      particleFilterState = PROCESSSCAN;

      PFLocMutex.unlock(); 
   }
   
     // debug.
//   ++odomCounter;
   
}

//*************************************************************************************************
//  Function: Odom Callback
//           This function updates the particle filter with the latest laser scan message.
//
//  Parameters scan an LaserScan reading from the robot.
//
//  returns: none.   
//
//*************************************************************************************************
void scanCallback(LaserScan scan)
{
   
//   ROS_INFO("PFLocalizationNode::%s scanCounter = %u\n", __func__, scanCounter);
    
   if (particleFilterState == PROCESSSCAN)
   {
      // change the state to  PROCESSINGSCAN in case this function is not thread safe. 
      particleFilterState = PROCESSINGSCAN;
      PFLocMutex.lock();
         
      ROS_INFO("PFLocalizationNode::%s PROCESSINGSCAN \n", __func__); 

      // this seems like a strange place to put this but we're doing a bunch of
      // particle weight calcs in updateParticleSensorData so this is the best time
      // to remove any particles outside the area before we do those calculations.
      
      // Process QR codes;
      if(qRcodePoseRecieved == true && reinitializeParticleFilter == false)
      {
         ROS_INFO("PFLocalizationNode::%s prune particles \n", __func__); 
         particleFilter->QrCodePosePruneParticles();
         
         qRcodePoseRecieved = false;
      }
      
      particleFilter->updateParticleSensorData(scan);
   
      // go back to processing the odom.
      particleFilterState = PROCESSODOM;
      PFLocMutex.unlock();
   }
   
   // Debug
//   ++scanCounter;
}


void qrcodePoseCallback(Pose qrCodePose)
{
   Pose lastQrCodePose = particleFilter->getQrCodePose();
   
//   ROS_INFO("PFLocalizationNode::%s Called sec %f\n", __func__, qrCodePose.header.stamp.toSec()); 
//   ROS_INFO("PFLocalizationNode::%s Called difference sec %f\n", __func__, (qrCodePose.header.stamp.toSec() - lastQrCodePose.header.stamp.toSec())); 
   
   // Setting this to 60 seconds;
   if((qrCodePose.header.stamp.toSec() - lastQrCodePose.header.stamp.toSec()) >= 5)
   {
      PFLocMutex.lock();
      
      reinitializeParticleFilter = true;
      
      particleFilter->setQrCodePose(qrCodePose);
      qRcodePoseRecieved = true;
      
      PFLocMutex.unlock();
   }
}

