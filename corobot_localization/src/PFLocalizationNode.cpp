#include <ros/ros.h>
#include <ros/types.h>
#include <nav_msgs/Odometry.h>

#include "corobot_common/GetCoMap.h"
#include "corobot_common/Pose.h"
#include "corobot_common/PoseArray.h"


#include "ParticleFilter.h"

using namespace ros;
using nav_msgs::Odometry;
using corobot_common::Pose;
using corobot_common::PoseArray;

void odomCallback(Odometry odom);

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

   ros::Subscriber sub = n.subscribe("odom", 100, odomCallback);
   
   ros::Publisher particlePublisher = n.advertise<corobot_common::PoseArray>("particleList", 1);;

   
   ros::service::waitForService("get_map");
   
   corobot_common::GetCoMap coMap;

   if (ros::service::call("get_map", coMap))
   {
//      ROS_INFO("Got the comap!\n");
//      ROS_INFO("coMap.response.map.info.height %d!\n", coMap.response.map.info.height);
//      ROS_INFO("coMap.response.map.info.width %d!\n", coMap.response.map.info.width);

      ParticleFilter* particleFilter = new ParticleFilter(coMap.response.map);
      
      particleFilter->initialize(10);
      
      ParticleFilter::ParticleList results = particleFilter->getParticleList();

      ROS_INFO("PFLocalizationNode::%s results.size = %lu\n", __func__, results.size());   

      PoseArray particles; 
      
      for (ParticleFilter::ParticleList::iterator it=results.begin(); it != results.end(); ++it)
      {
         particles.poses.push_back((it->pose));
      }
      particlePublisher.publish(particles);

   }
   else
   {
      ROS_INFO("No map love baby!");
   }
   
   ros::spin();
}

void odomCallback(Odometry odom)
{
   double x_m = odom.pose.pose.position.x;
   double y_m = odom.pose.pose.position.y;
   double z_m = 0;
   ROS_INFO("odomCallback called x = %f, y = %f\n", x_m, y_m);
   
   
}
