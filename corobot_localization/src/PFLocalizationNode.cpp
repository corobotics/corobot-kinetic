#include <ros/ros.h>
#include <ros/types.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

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

ros::Publisher particlePublisher;
ParticleFilter* particleFilter = NULL;

void odomCallback(Odometry odom);
void goalCallback(Point goal);

typedef enum {
  STOPPED,
  IDLE,
  INITIALIZING,
  INITIALIZED,
  RUNNING,
  PROCESSINGODOM,
  STATEMAX
} ParticleFilterState;

// if we recieve a goal we start running the particle filter. 
ParticleFilterState particleFilterState = STOPPED;
static unsigned int debugIndex = 0;


Odometry testVector[8];

std::mutex PFLocMutex;

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

   //ros::Subscriber odomSub = n.subscribe("odom", 10, odomCallback);
   ros::Subscriber odomSub = n.subscribe("todom", 10, odomCallback);
   ros::Subscriber goalSub = n.subscribe("goals_nav", 1, goalCallback);
   
   particlePublisher = n.advertise<corobot_common::PoseArray>("particleList", 1);;

   
   ros::service::waitForService("get_map");
   
   corobot_common::GetCoMap coMap;

   if (ros::service::call("get_map", coMap))
   {
//      ROS_INFO("Got the comap!\n");
//      ROS_INFO("coMap.response.map.info.height %d!\n", coMap.response.map.info.height);
//      ROS_INFO("coMap.response.map.info.width %d!\n", coMap.response.map.info.width);

      particleFilter = new ParticleFilter(coMap.response.map, 0.5, 60);
/*      
      testVector[0].pose.pose.position.y = 0.1; // OK counterclock
      testVector[0].pose.pose.position.x = 0;
      
      testVector[1].pose.pose.position.y = 0.2; // OK counterclock
      testVector[1].pose.pose.position.x = 0;
      
      testVector[2].pose.pose.position.y = 0.3; // OK counterclock
      testVector[2].pose.pose.position.x = 0;
      
      testVector[3].pose.pose.position.y = 0.4; // OK counterclock
      testVector[3].pose.pose.position.x = 0;
      
      testVector[4].pose.pose.position.y = 0.5; // OK counterclock
      testVector[4].pose.pose.position.x = 0;
      
      testVector[5].pose.pose.position.y = 0.6; // OK counterclock
      testVector[5].pose.pose.position.x = 0;

      testVector[6].pose.pose.position.y = 0.7; // OK counterclock
      testVector[6].pose.pose.position.x = 0;
      
      testVector[7].pose.pose.position.y = 0.8; // OK counterclock
      testVector[7].pose.pose.position.x = 0;
      */
      
      testVector[0].pose.pose.position.x = 1.0; // OK counterclock
      testVector[0].pose.pose.position.y = 0;
      testVector[0].pose.pose.orientation.z = 0;
      testVector[0].pose.pose.orientation.w = 0;
      
      testVector[1].pose.pose.position.x = 1; // OK counterclock
      testVector[1].pose.pose.position.y = 1;
      testVector[1].pose.pose.orientation.z = 0;
      testVector[1].pose.pose.orientation.w = 0;
      
      testVector[2].pose.pose.position.x = 0;
      testVector[2].pose.pose.position.y = 1;
      testVector[2].pose.pose.orientation.z = 0;
      testVector[2].pose.pose.orientation.w = 0;
      
      testVector[3].pose.pose.position.x = -1;
      testVector[3].pose.pose.position.y = 1;
      testVector[3].pose.pose.orientation.z = 0;
      testVector[3].pose.pose.orientation.w = 0;
      
      testVector[4].pose.pose.position.x = -1;
      testVector[4].pose.pose.position.y = 0;
      testVector[4].pose.pose.orientation.z = 0;
      testVector[4].pose.pose.orientation.w = 0;
      
      testVector[5].pose.pose.position.x = -1;
      testVector[5].pose.pose.position.y = -1;
      testVector[5].pose.pose.orientation.z = 0;
      testVector[5].pose.pose.orientation.w = 0;
      
      testVector[6].pose.pose.position.x = 0;
      testVector[6].pose.pose.position.y = -1;
      testVector[6].pose.pose.orientation.z = 0;
      testVector[6].pose.pose.orientation.w = 0;
      
      testVector[7].pose.pose.position.x = 1;
      testVector[7].pose.pose.position.y = -1;
      testVector[7].pose.pose.orientation.z = 0;
      testVector[7].pose.pose.orientation.w = 0;


      particleFilterState = IDLE;
   }
   else
   {
      ROS_INFO("No map love baby!");
   }
   
   ros::spin();
}

void goalCallback(Point goal)
{
   ROS_INFO("goalCallback called x = %f, y = %f\n", goal.x, goal.y);
      // If we recieved a new goal reinitialze the particle filter.
   if(particleFilterState == IDLE || particleFilterState == RUNNING)
   {
     
      
      particleFilterState = INITIALIZING;
      
      PFLocMutex.lock();

      particleFilter->initialize(100);
      
//      particleFilter->initialize(10, odom);
      debugIndex = 0;
      
      PFLocMutex.unlock();      
            
      ROS_INFO("PFLocalizationNode::%s initialization completed\n", __func__);
 
      particleFilterState = RUNNING;

   } 
   else
   {
      ROS_INFO("odomCallback ERROR: recieved a new goal when in state %d\n", particleFilterState);
   }
}

void odomCallback(Odometry odom)
{
   static bool sendParticles = false;
   
   double x_m = odom.pose.pose.position.x;
   double y_m = odom.pose.pose.position.y;
   double z_m = 0;
  
   

   if (particleFilterState == RUNNING)
   {
      particleFilterState = PROCESSINGODOM;
      ROS_INFO("PFLocalizationNode::%s PROCESSINGODOM x = %f, y = %f\n", __func__, x_m, y_m); 
      PFLocMutex.lock();
      
      PoseArray particles; 

 //     ROS_INFO("PFLocalizationNode::%s updateParticlePositions start\n", __func__); 
//      particleFilter->updateParticlePositions(testVector[(debugIndex % 8)]);
      particleFilter->updateParticlePositions(odom);
 //     ROS_INFO("PFLocalizationNode::%s updateParticlePositions end\n", __func__);
            

      ++debugIndex;
      if(sendParticles)
      {
         ParticleFilter::ParticleList& results = particleFilter->getParticleList();
         
         for (ParticleFilter::ParticleList::iterator it=results.begin(); it != results.end(); ++it)
         {
   //         ROS_INFO("PFLocalizationNode::%s it->pose x = %f, y = %f\n", __func__, it->pose.x, it->pose.y);
            particles.poses.push_back((it->pose));
         }
         
          ROS_INFO("PFLocalizationNode::%s particles = %lu\n", __func__, particles.poses.size());
         
         particlePublisher.publish(particles);
         
         sendParticles = false;
      }
      else
      {
         sendParticles = true;
      }
      
      PFLocMutex.unlock();  
                  
      particleFilterState = RUNNING;
   }
}
