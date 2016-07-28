#include <cmath>
#include <iostream>
#include <math.h>

#include "ros/console.h"
#include "corobot_common/Pose.h"
#include "corobot.h"
#include "apf.h"
#include "wall_detection.h"

using corobot::bound;
using corobot::length;
using corobot::rCoordTransform;
using corobot_common::Pose;
using sensor_msgs::LaserScan;

bool Polar::isValid() const {
    return d >= 0.0;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::scanToList(LaserScan scan) {
    list<Polar> points;
    // Create a placeholder with the initial angle.
    Polar p;
    p.a = scan.angle_min;
    // Convert scan.ranges into a list.
    for (unsigned int i = 0; i < scan.ranges.size() - 1; i++) {
        float d = scan.ranges[i];
        if (d > scan.range_min && d < scan.range_max) {
            p.d = d;
        } else {
            // New convention: < 0 means invalid.
            p.d = -1.0;
        }
        points.push_back(p);
        p.a += scan.angle_increment;
    }
    return points;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::findLocalMinima(list<Polar> points) {
    // List of local minima that have been found.
    list<Polar> localMinima;
    // Whether the last angle's distance was smaller than the current one.
    bool lastWasCloser = false;
    // The previous point; init to an invalid point.
    Polar prev = {-1.0, 0.0};
    for (list<Polar>::iterator i = points.begin(); i != points.end(); i++) {
        Polar p = *i;
        // If p is a valid point and closer than the previous one.
        if (p.isValid() && (!prev.isValid() || p.d < prev.d)) {
            // We mark it as a potential candidate for being a local minima.
            lastWasCloser = true;
        } else {
            // Otherwise if i-1 was closer than i-2, i-1 is a local minima.
            if (lastWasCloser) {
                localMinima.push_back(prev);
            }
            lastWasCloser = false;;
        }
        prev = p;
    }
    // Check in case the last point was a minima.
    if (lastWasCloser) {
        localMinima.push_back(prev);
    }
    int index = 0;
    for (list<Polar>::iterator i = localMinima.begin(); i != localMinima.end(); i++) {
        Polar p = *i;
        //ROS_DEBUG("minima%d:\t(%.2f, %.2f)", index, p.d, p.a);
        index = index + 1;
    }
    return localMinima;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::findObjects(list<Polar> points) {
    // List of "object" points: local minima of the scan.
    list<Polar> objects;
    // Point of the current object's minimum, for setting and adding to objects.
    Polar objMin;
    Polar last;
    last.d = -1.0;
    for (list<Polar>::iterator i = points.begin(); i != points.end(); i++) {
        Polar p = *i;
        if (p.d >= 0) {
            // if this is a valid point
            if (last.d >= 0) {
                // and there is an obj in progress
                if (abs(p.d - last.d) < 0.2) {
                    // check if this point is still in the object
                    if (p.d < objMin.d) {
                        // if this point is closer than objMin, save it.
                        objMin = p;
                    }
                } else {
                    // not in an object; add the previous object to the list and
                    // make a new one.
                    objects.push_back(objMin);
                    objMin = p;
                }
            } else {
                // no object in progress; start a new one.
                objMin = p;
            }
        } else if (last.d >= 0) {
            // else if there was an object in progress, add it to the list.
            objects.push_back(objMin);
        }
        last = p;
    }
    if (last.d >= 0) {
        objects.push_back(objMin);
    }
    int index = 0;
    for (list<Polar>::iterator i = objects.begin(); i != objects.end(); i++) {
        Polar p = *i;
        //ROS_DEBUG("object%d=(%.2f, %.2f)", index, p.d, p.a);
        index += 1;
    }

    ROS_DEBUG("objects=%d", objects.size());
    return objects;
}

// ruchin
list<Polar> scanToList_noWallPoints(LaserScan scan, Wall wall){
    list<Polar> points;
    // Create a placeholder with the initial angle.
    Polar p;
    p.a = scan.angle_min;
    // Convert scan.ranges into a list.
    for (unsigned int i = 0; i < scan.ranges.size() - 1; i++) {
        float d = scan.ranges[i];
        if (d > scan.range_min && d < scan.range_max) {
            bool nearby = false;
            float ang = scan.angle_min + (i*scan.angle_increment);
            float x = scan.ranges[i]*cos(ang);
            float y = scan.ranges[i]*sin(ang);
            float lineDist;
            if (wall.is_wall_left){
                lineDist = abs((100.0*x*cos(wall.thetaleft)) + (100.0*y*sin(wall.thetaleft)) - wall.rleft);
                if (lineDist <= 25.00)
                    nearby = true;
            }
            if (wall.is_wall_right){
                lineDist = abs((100.0*x*cos(wall.thetaright)) + (100.0*y*sin(wall.thetaright)) - wall.rright);
                if (lineDist <= 25.00)
                    nearby = true;
            }
            if (nearby)
                p.d = -1.0;
            else
                p.d = d;
        } else {
            // New convention: < 0 means invalid.
            p.d = -1.0;
        }
        points.push_back(p);
        p.a += scan.angle_increment;
    }
    return points;
}
// /ruchin

/**
 * {@inheritDoc}
 * Navigates using an artificial potential field technique with cached
 * obstacles to assist with the narrow field of view of the Kinect.
 * Order of operations:
 * 1. Compute goal location relative to robot
 * 2. Compute goal force: calcGoalForce
 * 3. Turn Kinect scan into list of discrete obstacles: findObjects
 * 4. Updates obstacle cache - delete old/far-away obstacles, 
 *     add new obstacles: updateObstacleList
 * 5. Add force due to obstacles: updateNetForce
 * 6. Convert (x,y) force vector to polar (relative to robot heading)
 * 7. Convert force vector to command velocity: cmdTransform
 * 8. Check for timeout to reach current waypoint
 * 9. Check for recovery (is robot lost) - not sure this check is
 *      being done properly at present
 */
Polar APF::nav(LaserScan scan) {
    // Can't do anything without a goal.
    if (waypointQueue.empty()) {
        // ROS_INFO("No waypoints in queue!");
        Polar p; p.a = 0; p.d = 0;
        return p;
    }

    if(timeDeployed == -1){
        ROS_INFO("deploying at time=%.2f", ros::Time::now().toSec());
        timeDeployed = ros::Time::now().toSec(); 
    }
    else
        ROS_INFO("current time=%.2f", ros::Time::now().toSec());

    stringstream ss; corobot_common::Goal topicMsg;

    // ruchin
    corobot_common::Goal pathcolor;
    double x1,y1,x2,y2;
    Polar vel;
    float v,w;
    // WallDetector *wd = new WallDetector();
    Wall currWall = wd->houghTransform(scan);
    wallPublisher.publish(currWall);

    bool followL=false, followR=false, follow=true, slowFollow=false;

    Point nextGoal = waypointQueue.front();
    Polar* goalwrtrobot = convertFromGlobalToRobotInPolar(nextGoal);
    float thetagoal = goalwrtrobot->a;
    float dist = goalwrtrobot->d;
    float thetawall, rwall;
    thetagoal = thetagoal*180.00/M_PI;
    thetagoal = fmod(thetagoal,360.0);
    if (thetagoal < 0)
        thetagoal += 360.00;
    thetagoal += 90.00;
    thetagoal = fmod(thetagoal,360.0);

    if (currWall.is_wall_left && abs(currWall.thetaleft - thetagoal) <= 45.00){
        followL = true;
    }
    if (currWall.is_wall_right && abs(currWall.thetaright - thetagoal) <= 45.00){
        followR = true;
    }

    if (dist >= 1.5 || inOneLine){
        if (lastWallL){
            if (followL){
                thetawall = currWall.thetaleft;
                rwall = currWall.rleft;
            }
            else if(followR){
                thetawall = currWall.thetaright;
                rwall = currWall.rright;
                lastWallR = true;
                lastWallL = false;
            }
            else{
                lastWallR = lastWallL = false;
                follow = false;
            }
        }
        else if(lastWallR){
            if (followR){
                thetawall = currWall.thetaright;
                rwall = currWall.rright;
            }
            else if(followL){
                thetawall = currWall.thetaleft;
                rwall = currWall.rleft;
                lastWallR = false;
                lastWallL = true;
            }
            else{
                lastWallR = lastWallL = false;
                follow = false;
            }   
        }
        else if(followL || followR){
            if (followL && followR){
                thetawall = (currWall.conf_right > currWall.conf_left)?(currWall.thetaright):(currWall.thetaleft);
                rwall = (currWall.conf_right > currWall.conf_left)?(currWall.rright):(currWall.rleft);
                lastWallL = (currWall.conf_right > currWall.conf_left)?(false):(true);
                lastWallR = !lastWallL;
            }
            else if(followL){
                thetawall = currWall.thetaleft;
                rwall = currWall.rleft;
                lastWallL = true;
                lastWallR = false;
            }
            else{
                thetawall = currWall.thetaright;
                rwall = currWall.rright;
                lastWallL = false;
                lastWallR = true;
            }
        }
        else{
            lastWallL = lastWallR = false;
            follow  = false;
        }
        if (!follow){
            if (currWall.is_wall_left && currWall.is_wall_right){
                    if (abs(currWall.thetaleft - currWall.thetaright) <= 10){
                        ROS_INFO("Cant seem to find any navigable wall but it seems there are walls parallel on both sides...");
                        ROS_INFO("Hence, slowly following closer wall for now...");
                        thetawall = (abs(currWall.rleft) > abs(currWall.rright))?(currWall.thetaright):(currWall.thetaleft);
                        rwall = (abs(currWall.rleft) > abs(currWall.rright))?(currWall.rright):(currWall.rleft);
                        slowFollow = true;
                        follow = false;
                    }
            }
        }
        rwall = abs(rwall);
        rwall /= 100.00;
        // follow = false;
        // slowFollow = false;

        // Find objects in view not considering points nearby detected walls
        list<Polar> obsInView = findLocalMinima(findObjects(scanToList_noWallPoints(scan,currWall)));
        // Check if there is an object nearby
        // If there is one, use APF to navigate
        // Otherwise, just go ahead and use walls (if there are any!)
        if (slowFollow || follow){
            for (list<Polar>::iterator it = obsInView.begin(); it != obsInView.end(); ++it) {
                Polar obsWrtRobot = *it;
                if (obsWrtRobot.d <= D_OBS) {
                    ROS_INFO("Found obstacle(s) nearby other than the walls....");
                    ROS_INFO("Using APF for now...");
                    slowFollow = false;
                    follow = false;
                }
            }
        }
        for (int i=0;i<7;i++){
            float x1 = intersections[i][0];
            float y1 = intersections[i][1];
            float range = sqrt((pose.x-x1)*(pose.x-x1) + (pose.y-y1)*(pose.y-y1));
            if (range <= 3.0 ){
                ROS_INFO("Approaching intersection...using APF for now...");
                slowFollow = false;
                follow = false;
                break;
            }
        }
        if (waypointQueue.size() <= 1 && goalwrtrobot->d <= 3.00){
            ROS_INFO("Within 3 meters of final goal...using APF for now...");
            slowFollow = false;
            follow = false;
        }
        if (follow || slowFollow){
            ROS_INFO("Using wall now...");
            ROS_INFO("Wall theta is: %f",thetawall);
            ROS_INFO("Wall distance is: %f",rwall);
            float thetaRange;

            if (follow)
                v = 0.3;
            else
                v = 0.3;   
            v = bound(v,cmdPrev.d,0.01);
            cmdPrev.d = v;  
            w = ((thetawall - 90.00) * v) / (rwall - SET_DIST);
            w = w * M_PI / 180.00;

            if (rwall <= 0.5)
                w = (w < 0)?(w-0.1):(w+0.1);

            cout << "Translation velocity: " << v << "\n";
            cout << "Rotational velocity: " << w << "\n";

            vel.d = v; 
            vel.a = w;

            
            x1 = pose.x; x2 = prevRobotPose.x;
            y1 = pose.y; y2 = prevRobotPose.y;

            if(startClock < 0){
                startClock = ros::Time::now().toSec(); 
            }

            if (distanceTraveled < 0.00)
                distanceTraveled = 0.00;
            else{
                if (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) < 0.1)
                    distanceTraveled += sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
            }


            journeyTime = (scan.header.stamp.toSec() - startClock);

            prevRobotPose.x = pose.x; prevRobotPose.y = pose.y; prevRobotPose.a = pose.theta;



            //ROS_INFO("Position theta is: %f",180.0*pose.theta/pi);
            //ROS_INFO("Wall theta is: %f",thetawall);
            //ROS_INFO("Goal's theta is: %f",thetagoal-90.00);
            //ROS_INFO("Goal's theta is: %f",thetagoal);
            //ROS_INFO("Difference is: %f",abs(thetawall - thetagoal));
            //ROS_INFO("Waypoint distance is: %f",dist);
            //ROS_INFO("Last wall left? %d",lastWallL);
            //ROS_INFO("Follow left? %d",followL);
            //ROS_INFO("Last wall right? %d",lastWallR);
            //ROS_INFO("Follow right? %d",followR);
            //ROS_INFO("Distance traveled: %f meters.",distanceTraveled);
            //ROS_INFO("Time taken: %f seconds.",journeyTime);


            Point goalInMap = waypointQueue.front();
            //ROS_DEBUG("Abs Goal:\t(%.2f, %.2f)", goalInMap.x, goalInMap.y);
            ss.str(""); ss << "(" << goalInMap.x << ", " << goalInMap.y << ")";
            topicMsg.name = ss.str(); absGoalPublisher.publish(topicMsg);
            if (inOneLine){
                pathcolor.name = "green";
            }
            else{
                if (slowFollow)
                    pathcolor.name = "red";
                else
                    pathcolor.name = "red";
            }
            wallfPublisher.publish(pathcolor);
            return vel;
        }
        else{
            ROS_INFO("Waypoint not approachable by any wall...");
        }
        // ROS_INFO("Position theta is: %f",180.0*pose.theta/pi);
        // ROS_INFO("Wall theta is: %f",thetawall);
        // ROS_INFO("Goal's theta is: %f",thetagoal-90.00);
        // ROS_INFO("Goal's theta is: %f",thetagoal);
        // ROS_INFO("Difference is: %f",abs(thetawall - thetagoal));
        // ROS_INFO("Waypoint distance is: %f",dist);
        // ROS_INFO("Last wall left? %d",lastWallL);
        // ROS_INFO("Follow left? %d",followL);
        // ROS_INFO("Last wall right? %d",lastWallR);
        // ROS_INFO("Follow right? %d",followR);
    }
    else{
        lastWallL = lastWallR = false;
        ROS_INFO("Approaching waypoint...using APF for now...");
    }
    ROS_INFO("Not using wall now...");
    //ROS_WARN("WayPoint curr length: %d", waypointQueue.size());
    // /ruchin

    lastScanTime = scan.header.stamp.toSec();
    angleMin = scan.angle_min;
    angleMax = scan.angle_max;
    float delta = length((prevRobotPose.x-pose.x), (prevRobotPose.y-pose.y));
    if(delta < .1){
        distanceTraveled += length((prevRobotPose.x-pose.x), (prevRobotPose.y-pose.y));
    }
    ROS_DEBUG("Pose:\t(%.2f, %.2f), <%.2f>", pose.x, pose.y, pose.theta);
    // ROS_DEBUG("Traveled Distance: %.2f", distanceTraveled);
    // ROS_DEBUG("Travel Time: %.2f", scan.header.stamp.toSec() - timeDeployed);

    if(inRecovery){
        //capture things for the next cycle
        // prevRobotPose.x = pose.x; prevRobotPose.y = pose.y; prevRobotPose.a = pose.theta; 
        ROS_INFO("about to recover");
        return doRecoveryNav(scan);
    }

    ROS_INFO("Not recovering");
    // ruchin
    pathcolor.name = "blue";
    wallfPublisher.publish(pathcolor);
    // /ruchin

    //clear obstacleList for if the robot sees a barcode and there is a difference in odometer and the qrcode's localization
    if(abs(prevRobotPose.x - pose.x) > 1.0 || abs(prevRobotPose.y - pose.y) > 1.0 || abs(prevRobotPose.a - pose.theta) > 1.0){
        activeObstacleList.clear();

        ss.str(""); ss << "obs cleared: " << activeObstacleList.size();
        topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);
        ROS_INFO("Clearing ObstacleList: %u", activeObstacleList.size());
    }

    // The goal is the head of the waypoint queue.
    Point goalInMap = waypointQueue.front();
    //ROS_DEBUG("Abs Goal:\t(%.2f, %.2f)", goalInMap.x, goalInMap.y);
    ss.str(""); ss << "(" << goalInMap.x << ", " << goalInMap.y << ")";
    topicMsg.name = ss.str(); absGoalPublisher.publish(topicMsg);

    // Convert the goal into the robot reference frame.
    Point goalWrtRobot = rCoordTransform(goalInMap, pose);
    //ROS_DEBUG("Rel Goal:\t(%.2f, %.2f)", goalWrtRobot.x, goalWrtRobot.y);

    // The list of "objects" found; already in the robot reference frame.
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));

    updateObstacleList(objects);
    
    // Resulting command vector.
    Polar cmdInitial;
    // binit FMD
    // cmdInitial = getFeasibleMotionDirection(goalWrtRobot);
    cmdInitial.a = 0.0;
    if (cmdInitial.a == 0.0) { 
        // if no change in angle means no feasible motion directions available, continue using APF
    // /binit FMD    
        // Stores the object force vector summation. z is ignored.
        Point netForce = calcGoalForce(goalWrtRobot, lastScanTime);

        ROS_DEBUG("Net force:\t<%+.2f, %.2f>",netForce.x,netForce.y);
        cmdInitial.d = length(netForce.x, netForce.y);
        if(abs(netForce.y) <= 0.099 && abs(netForce.x) <= 0.099) {
            //ROS_DEBUG("zero net force detected");
            cmdInitial.a = 0;
        }else {
            cmdInitial.a = atan2(netForce.y, netForce.x);
        }
    }

    ROS_DEBUG("Raw cmd:\t<%+.2f, %.2f>", cmdInitial.d, cmdInitial.a);
    // publishing raw navigation command to the ch_rawnav topic
    ss.str(""); ss << "<" << cmdInitial.d << ", " << cmdInitial.a << ">";
    topicMsg.name = ss.str(); rawnavPublisher.publish(topicMsg);

    Polar cmd = cmdTransform(cmdInitial);

    ROS_DEBUG("Given cmd:\t<%+.2f, %.2f>", cmd.d, cmd.a);

    double now = lastScanTime;
    if (cmd.d > 0.0 || timeLastMoved == 0.0) {
        timeLastMoved = now;
    } else if (now - timeLastMoved > 10.0) {
        // Haven't moved in too long; give up on this waypoint.
        waypointQueue.pop();
        // ruchin
        waypointQVector.erase(waypointQVector.begin());
        // /ruchin
        failedQueue.push(goalInMap);
        // Reset the timestamp so we don't give up on subsequent waypoints.
        timeLastMoved = 0.0;
    }

    if(waypointQueue.size() != 0)
        recoveryCheck(scan.header.stamp.toSec());
 
    // tony python
    if (cmd.d < 0.0)
        cmd.d = 0.0;

    //capture things for the next cycle
    cmdPrev = cmd;

    // ruchin
    x1 = pose.x; x2 = prevRobotPose.x;
    y1 = pose.y; y2 = prevRobotPose.y;
    

    if(startClock < 0){
        startClock = ros::Time::now().toSec(); 
    }

    if (distanceTraveled < 0.00)
        distanceTraveled = 0.00;
    else{
        if (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) < 0.5)
            distanceTraveled += sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }


    journeyTime = (scan.header.stamp.toSec() - startClock);


    ROS_INFO("Distance traveled: %f meters.",distanceTraveled);
    ROS_INFO("Time taken: %f seconds.",journeyTime);
    // /ruchin

    prevRobotPose.x = pose.x; prevRobotPose.y = pose.y; prevRobotPose.a = pose.theta;

    // tony
    /*doorDist = sqrt((self.pose.x - 86.9)**2 + (self.pose.y - 35.1)**2)
    //if doorDist < .5:
    if (self.i == 100)
        self.i = 0;
    // rospy.sleep(rospy.Duration(10))
    self.publishers['repForce'].publish(name = 'clear')
    self.publishers['perpForce'].publish(name = 'clear')
    // if doorDist < .5:
    self.i += 0; */
    // /tony
    return cmd;
}

/**
 * Convert the given polar (relative to robot) coordinate to the
 * global coordinate system.
 */
corobot::SimplePose* APF::convertRobotToGlobal(Polar &polarPoint){
    corobot::SimplePose *sp = new corobot::SimplePose();
    sp->x = pose.x + polarPoint.d * cos(pose.theta + polarPoint.a);
    sp->y = pose.y + polarPoint.d * sin(pose.theta + polarPoint.a);
    sp->a = 0;
    //ROS_DEBUG("robot=(%.2f,%.2f) -> global=(%.2f, %.2f)",polarPoint.d, polarPoint.a, sp->x, sp->y);
    //ROS_DEBUG("APF, Returning RToG Coordinates");
    return sp;
}

/**
 * Add the given point to the obstacle cache if it is farther than
 * OBS_MATCH_DIST away from all other cached obstacles.
 */
bool APF::pushIfUnique(corobot::SimplePose *sp){
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obsPt = it->p;
        if(length(sp->x - obsPt.x,sp->y - obsPt.y) <= OBS_MATCH_DIST) { //if the point approx matches the points in the list
            //ROS_DEBUG("APF, PushUnique returns False");
            it->lastT = lastScanTime;
            return false;
        }
    }
    
    //ROS_DEBUG("APF, Pushing Object into activeObstacleList: %.2f, %.2f", sp->x, sp->y);
    CachedPoint newObs;
    newObs.p.x = sp->x;
    newObs.p.y = sp->y;
    newObs.lastT = lastScanTime;
    newObs.lastT = lastScanTime;
    activeObstacleList.push_back(newObs);

    //publishing the obstacle's coordinates to the ch_obstacle topic
    stringstream ss; corobot_common::Goal topicMsg;
    ss << "(" << sp->x << ", " << sp->y << "), add : " << activeObstacleList.size();
    topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);

    //ROS_DEBUG("APF, current list Size: %u", activeObstacleList.size());
    return true;
}

double APF::distanceFromRobot(corobot::SimplePose &sp){
    return sqrt((sp.x-pose.x)*(sp.x-pose.x) + (sp.y-pose.y)*(sp.y-pose.y));
}

/**
 * Convert the given point in the global coordinates to the robot's
 * relative coordinate system.
 */
Polar* APF::convertFromGlobalToRobotInPolar(Point &sp){
    Polar *p = new Polar();
    if(abs(sp.y-pose.y) <= 0.099 && abs(sp.x - pose.x) <= 0.099){
        p->a = 0;
        p->d = 0;
        //ROS_DEBUG("APF, Returning GToR: zeros detected");
    }
    else{ 
        p->a = atan2(sp.y-pose.y, sp.x - pose.x) - pose.theta;
        p->d = sqrt((sp.x-pose.x)*(sp.x-pose.x) + (sp.y-pose.y)*(sp.y-pose.y));
    }
    //ROS_DEBUG("APF, Returning GToR coordinates");
    return p;
}

double APF::min(double a, double b){
    if(a<=b) 
        return a;
    return b;
}

/**
 * Convert a force vector into fwd/angular velocity
 * cmd.d is forward vel command, cmd.a is angular vel command
 */
Polar APF::cmdTransform(Polar &cmdInitial){
    Polar cmd;
    // Don't try to go forward if the angle is more than fixed value.
    if (cmdInitial.a > ANGLE_WINDOW) {
        
        if (cmdInitial.a < M_PI/2.0) {
            cmd.d = cos(cmdInitial.a)*min(cmdInitial.d,MAX_VEL);
            cmd.a = MIN_OMEGA + (MAX_OMEGA-MIN_OMEGA)*
                          (cmdInitial.a-ANGLE_WINDOW)/(M_PI/2.0-ANGLE_WINDOW);
        } else {
            cmd.d = 0;
            cmd.a = MAX_OMEGA;
        }
    } else if (cmdInitial.a < -ANGLE_WINDOW) {
        if (cmdInitial.a > -M_PI/2.0) {
            cmd.d = cos(cmdInitial.a)*min(cmdInitial.d,MAX_VEL);
            cmd.a = -MIN_OMEGA - (MAX_OMEGA-MIN_OMEGA)*
                           (ANGLE_WINDOW-cmdInitial.a)/(ANGLE_WINDOW+M_PI/2.0);
        } else {
            cmd.d = 0;
            cmd.a = -MAX_OMEGA;
        } 
    } else {
        // if force is near straight, just head straight for now.
        cmd.a = 0;
        // forward velocity equal to force (for now, then gets capped below)
        cmd.d = cmdInitial.d;
    }

    if(cmd.d > MAX_VEL)
        cmd.d = MAX_VEL;
    cmd.d = bound(cmd.d, cmdPrev.d, 0.050);

    stringstream ss; corobot_common::Goal topicMsg;
    ss << "<" << cmd.d << ", " << cmd.a << ">";
    topicMsg.name = ss.str(); velCmdPublisher.publish(topicMsg);
    //ROS_DEBUG("NavVel:\t<%+.2f, %.2f>\n", cmd.d, cmd.a);
    
    return cmd;
}

// binit
double det(double m[][3]){
    double a = m[0][0]; double b = m[0][1]; double c = m[0][2];
    double d = m[1][0]; double e = m[1][1]; double f = m[1][2];
    double g = m[2][0]; double h = m[2][1]; double i = m[2][2];
    return (a * e * i) - (a * f * h) + (b * f * g) - (b * d * i) + (c * d * h) - (c * e * g);
}
// /binit

/**
 * Compute the goal force for the given relative goal location.
 */
Point APF::calcGoalForce(Point &goalWrtRobot, double now){
    Point netForce;
    /*
    ################
    # attractive F #
    ################
    */
    double goalDist = length(goalWrtRobot.x, goalWrtRobot.y);
    if (goalDist <= D_GOAL) {
        netForce.x = (K_GOAL / D_GOAL) * goalWrtRobot.x;
        netForce.y = (K_GOAL / D_GOAL) * goalWrtRobot.y;
    } else {
        netForce.x = K_GOAL * goalWrtRobot.x / goalDist;
        netForce.y = K_GOAL * goalWrtRobot.y / goalDist;
    }
    ROS_DEBUG("GoalF:\t%.2f, %.2f", netForce.x, netForce.y);
    // return netForce;
// }

/**
 * Add obstacle force from all cached obstacles to the given (goal) 
 * currently active force.
 */
// void APF::updateNetForce(Point &netForce, double now){
    stringstream ss; corobot_common::Goal topicMsg;
    int objIndex = 0;
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point repForce;
        // if (now - it->firstT < 0.2) ROS_DEBUG("Phantom?"); //continue; // skip new in case phantoms
        /*
        ################
        # repulsive  F #
        ################
        */
        Polar *od = convertFromGlobalToRobotInPolar(it->p);
        //if (od->d < 0.1) continue;
        //Point perpF; // to the right of netForce
        //perpF.x = netForce.y; perpF.y = -1*netForce.x; 
        double fmag = K_OBS * (1.0/D_OBS - 1.0/od->d) / (od->d * od->d);
        //double dtheta = od->a - origTheta;
        //while (dtheta < -M_PI) dtheta += 2*M_PI;
        //while (dtheta > M_PI) dtheta -= 2*M_PI;
        //if (dtheta > 0) fmag *= -1;
        repForce.x = fmag * cos(od->a);
        repForce.y = fmag * sin(od->a);
        netForce.x += repForce.x; //fmag * cos(dtheta) * perpF.x; 
        netForce.y += repForce.y; //fmag * cos(dtheta) * perpF.y;
        ROS_INFO("APF, Obj(%d)F:\t%.2f, %.2f", objIndex, fmag * cos(od->a), fmag *sin(od->a));
        /*
        ################
        # experimental #
        ################
        */
        double theta = atan2(goalWrtRobot.y, goalWrtRobot.x) * 180.0; // convert into degrees // atan2(obj.y - self.pose.y, obj.x - self.pose.x)
        double delta = fmod((abs(pose.theta - od->a) * 180.0/M_PI), 180.0);
        int kmax = 10; // .8obs 6 kmax, tau 10
        int tau = 10;
        double repF[2][1] = {{repForce.x}, {repForce.y}};
        double M_RS[2][2] = {{0.0, -1.0}, {1.0, 0.0}};
        double M_LS[2][2] = {{0.0, 1.0}, {-1.0, 0.0}};
        double d = fmod((od->a * 180.0), 360.0); // convert angle into degrees
        double deltadeg = min(d, 360.0-d);
        //check left or right side
        ///// ToDo
        double A[3] = {1, pose.x, pose.y};
        double B[3] = {1, pose.x + cos(od->a), pose.y + sin(od->a)};
        double C[3] = {1, goalWrtRobot.x, goalWrtRobot.y};
        double m[3][3];
        // m[0] = A; m[1] = B; m[2] = C;
        memcpy(m[0], A, 3 * sizeof(double));
        memcpy(m[1], B, 3 * sizeof(double));
        memcpy(m[2], C, 3 * sizeof(double));
        bool isRight = (det(m) > 0.0)? true : false;
        ///// /ToDo
        //self.publishers['debug'].publish('theta={};delta={}'.format(theta,delta))
        double K = (2.0 * kmax) / (1.0 + exp(delta / tau));
        double perpF[2][1];
        ///// ToDo
        if (isRight){
            double temp[2][2];
            temp[0][0] = K * M_RS[0][0];
            temp[0][1] = K * M_RS[0][1];
            temp[1][0] = K * M_RS[1][0];
            temp[1][1] = K * M_RS[1][1];
            perpF[0][0] = (temp[0][0] * repF[0][0]) + (temp[0][1] * repF[1][0]);
            perpF[1][0] = (temp[1][0] * repF[0][0]) + (temp[1][1] * repF[1][0]);
            //perpF = dot(dot(K, M_RS), repF);
        }
        else{
            double temp[2][2];
            temp[0][0] = K * M_LS[0][0];
            temp[0][1] = K * M_LS[0][1];
            temp[1][0] = K * M_LS[1][0];
            temp[1][1] = K * M_LS[1][1];
            perpF[0][0] = (temp[0][0] * repF[0][0]) + (temp[0][1] * repF[1][0]);
            perpF[1][0] = (temp[1][0] * repF[0][0]) + (temp[1][1] * repF[1][0]);
            //perpF = dot(dot(K, M_LS), repF);
        }
        ///// /ToDo
        // perpF angle and repF and perpF magnitudes
        double dist1 = sqrt((repF[0][0]) * (repF[0][0]) + (repF[1][0]) * (repF[1][0]));
        double dist2 = sqrt((perpF[0][0]) * (perpF[0][0]) + (perpF[1][0]) * (perpF[1][0]));
        double ang = atan2(perpF[1][0], perpF[0][0]);
        ROS_INFO("APF, Obj(%d)PerpF:\t%.2f, %.2f Theta:\t%.2f", objIndex, perpF[0][0], perpF[1][0], theta);
        //self.publishers['debug'].publish('dist2={}'.format(dist2))
        ss.str(""); ss << "(" << dist1 << ", " << ang << ")";
        topicMsg.name = ss.str(); perpForcePublisher.publish(topicMsg);
        
        ss.str(""); ss << "(" << dist1 << ", " << atan2(repF[1][0], repF[0][0]) << ")";
        topicMsg.name = ss.str(); repForcePublisher.publish(topicMsg);

        netForce.x += dist2 * cos(ang);
        netForce.y += dist2 * sin(ang);
        /*
        ################
        # end experim. #
        ################
        */
        objIndex += 1;
        //ROS_DEBUG("Obj(%d)F:\t%.2f, %.2f", ++objIndex, fmag * cos(dtheta) * perpF.x, fmag * cos(dtheta) * perpF.y);
    }
    ss.str(""); ss << "(" << netForce.x << ", " << netForce.y << ")";
    topicMsg.name = ss.str(); netForcePublisher.publish(topicMsg);      
    return netForce;
}

/**
 * Update the obstacle cache: 
 * - Eliminate any obstacles not seen in OBS_CACHE_TIMEOUT seconds
 * - Eliminate any obstacles farther than D_OBS away
 * - Eliminate any obstacles behind the robot (necessary?)
 * - Add any new unique obstacles
 * - Update time-last-seen of present obstacles already in cache
 */
void APF::updateObstacleList(list<Polar>& objects){
    stringstream ss; corobot_common::Goal topicMsg;
    int objIndex = 0;
    // First throw away old obstacles, or those behind us or far away.
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obs = it->p;
        double obsD = length(obs.x-pose.x,obs.y-pose.y);
        double obsAbsA = atan2(obs.y-pose.y,obs.x-pose.x);
        //ROS_DEBUG("obsAbsA = atan2(%.2f - %.2f, %.2f - %.2f) = %.2f",obs.y,pose.y,obs.x,pose.x,obsAbsA);
        double obsRelA = pose.theta - obsAbsA;
        while (obsRelA > 2*M_PI) obsRelA -= 2*M_PI;
        while (obsRelA <= 0) obsRelA += 2*M_PI;
        ROS_INFO("APF, Obj(%d) Distance:\t%.2f Angle:\t%.3f ", ++objIndex, obsD, obsRelA);

        if ((obsD > D_OBS) || (lastScanTime - it->lastT > OBS_CACHE_TIMEOUT) ||
           ((obsRelA > M_PI/2) && (obsRelA < 3*M_PI/2)) || (obsRelA >= angleMin && obsRelA <= angleMax)) {
            ss.str(""); ss << "(" << obs.x << ", " << obs.y << "), rem : " << objIndex;
            it = activeObstacleList.erase(it);
            
            //ROS_DEBUG("APF: Object(%d) removed", objIndex);
            topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);
            
            if(it == activeObstacleList.end())
                break;
        }
    }
    // now add in new ones if they are close enough to worry about.
    for (list<Polar>::iterator it = objects.begin(); it != objects.end(); ++it) {
        Polar pointWrtRobot = *it;
        if (pointWrtRobot.d <= D_OBS) {
            corobot::SimplePose *pointWrtGlobal = convertRobotToGlobal(pointWrtRobot);
            //ROS_DEBUG("Polar Object that might be added:\t(%.2f, %.2f)", pointWrtRobot.d, pointWrtRobot.a);
            //ROS_DEBUG("Object in global cood:\t(%.2f, %.2f)", pointWrtGlobal->x, pointWrtGlobal->y);
            pushIfUnique(pointWrtGlobal);
        }
    } 
    int index = 0;
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obsPt = it->p;
    //for (list<Polar>::iterator it = objects.begin(); it != objects.end(); ++it) {
    //    Polar pointWrtRobot = *it;
        //ROS_DEBUG("obstacle%d=(%.2f, %.2f)", index, obsPt.x, obsPt.y);
        index = index + 1;
    }
}

// binit FMD
/**
 * Find Feasible Motion Directions for each Obstacle
 * Find intersection of Feasible Motion Directions for each Obstacle
 * Return Feasible Direction(if any available) nearest to the Direction of Goal
 * Otherwise return zero angle(no change in direction)
 */
Polar APF::getFeasibleMotionDirection(Point &goalWrtRobot){
    // Feasible motion direction set
    double thetaMin = -M_PI;
    double thetaMax = M_PI;
    Polar feasibleDir;
    feasibleDir.d = 0.0;
    feasibleDir.a = 0.0;

    double goalDist = length(goalWrtRobot.x, goalWrtRobot.y); // distance to goal from robot
    double thetaGoal = atan2(goalWrtRobot.y, goalWrtRobot.x); // angle to goal wrt to direction of robot

    stringstream ss; corobot_common::Goal topicMsg;
    int objIndex = 0;
    // First throw away old obstacles, or those behind us or far away.
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obs = it->p;
        double obsD = length(obs.x-pose.x, obs.y-pose.y);
        double obsAbsA = atan2(obs.y-pose.y, obs.x-pose.x); // obsAbsA is in range [-PI, PI]
        ROS_INFO("FMD, Pose Theta\t%.3f ", pose.theta*180.0/M_PI);
        ROS_INFO("FMD, Obj(%d) Distance:\t%.2f Absolute Angle:\t%.3f ", ++objIndex, obsD, obsAbsA*180.0/M_PI);

        double obsRelA = pose.theta - obsAbsA; // obsRelA is in range [-2PI, 2PI]
        ROS_INFO("FMD, Obj(%d) Obstacle Relative Angle:\t%.3f ", objIndex, obsRelA*180.0/M_PI);

        while (obsRelA > M_PI) obsRelA -= 2*M_PI;
        while (obsRelA < -M_PI) obsRelA += 2*M_PI;
        // obsRelA is in range [-PI, PI]
        
        // feasible direction set for each obstacle
        double obsThetaMin, obsThetaMax;        
        if (obsRelA > 0) obsThetaMin = obsRelA - M_PI/2.0;
        else obsThetaMin = -M_PI/2.0;
        ROS_INFO("FMD, Obj(%d) Obstacle Theta Min Angle:\t%.3f ", objIndex, obsThetaMin*180.0/M_PI);

        if (obsRelA < 0) obsThetaMax = obsRelA + M_PI/2.0;
        else obsThetaMax = M_PI/2.0;
        ROS_INFO("FMD, Obj(%d) Obstacle Theta Max Angle:\t%.3f ", objIndex, obsThetaMax*180.0/M_PI);
        // feasible direction set is in range [-PI/2.0, PI/2.0]

        // shifting range from [-PI/2.0, PI/2.0] to [0, PI]
        obsThetaMin += M_PI/2.0;
        obsThetaMax += M_PI/2.0;
        // feasible direction set is in range [0, PI]
        
        // intersection of feasible direction sets
        if(obsThetaMin > thetaMin) thetaMin = obsThetaMin;
        if(obsThetaMax < thetaMax) thetaMax = obsThetaMax;
        // intersection of feasible direction sets is also in range [-PI/2.0, PI/2.0]
    }
    ////// to find optimal feasible direction closest to the negative gradient direction
    if (thetaMin > thetaMax) {
        // no Feasible Direction possible
        feasibleDir.d = 0.0;
        feasibleDir.a = 0.0;
        ROS_INFO("FMD, No Feasible Motion Direction Available... Using APF now");
    }
    else if (thetaMin <= thetaGoal && thetaGoal <= thetaMax) {
        // move in the direction of goal, goal direction is between thetaMin and thetaMax
        feasibleDir.d = goalDist/2.0;
        feasibleDir.a = thetaGoal;
        ROS_INFO("FMD, Feasible direction is the Goal Direction");
    }
    else if (abs(thetaGoal - thetaMin) < abs(thetaGoal - thetaMax)) {
        // thetaMin is closer to goal
        feasibleDir.d = 1.2;  //goalDist/2.0; // 1.2 is protective obstacle distance
        feasibleDir.a = thetaMin;
        ROS_INFO("FMD, Feasible direction is the Theta Min, Goal Distance:\t%.3f ", goalDist);
    }
    else {
        // thetaMax is closer to goal
        feasibleDir.d = 1.2;  //goalDist/2.0; // 1.2 is protective obstacle distance
        feasibleDir.a = thetaMax;
        ROS_INFO("FMD, Feasible direction is the Theta Max, Goal Distance:\t%.3f ", goalDist);
    }
    //////
    ROS_INFO("FMD, Feasible Theta Min Angle:\t%.3f ", thetaMin*180.0/M_PI);
    ROS_INFO("FMD, Feasible Theta Max Angle:\t%.3f ", thetaMax*180.0/M_PI);
    ROS_INFO("FMD, Feasible Direction:\t%.3f Feasible Angle:\t%.3f Goal Angle:\t%.3f ", feasibleDir.d, feasibleDir.a*180.0/M_PI, thetaGoal*180.0/M_PI);

    return feasibleDir;
}
// /binit FMD


/**
 * Supposedly used to check whether the robot needs to ignore the
 * potential field and go into recovery mode.
 */
void APF::recoveryCheck(const double &recov_time_now){
    if( ( waypointQueue.size() - prevWayPointQuelen ) == 0)
    {
       if( recov_time_now - timeSinceLastWayPoint > 60)
            recoverRobot();
    }
    else if( goal.x != 0 && goal.y != 0 && (pose.x > -.1 && pose.x < .1 ) && (pose.y > -.1 && pose.y < .1)){
        recoverRobot();
    }
    else
    {
         timeSinceLastWayPoint = recov_time_now;
         prevWayPointQuelen = waypointQueue.size();
         stringstream ss; corobot_common::Goal topicMsg;
         ss << "Not in Recovery ";
         topicMsg.name = ss.str(); recoveryPublisher.publish(topicMsg); 
    }
}

/**
 * Start the recovery process.
 */
void APF::recoverRobot(){
    activeObstacleList.clear(); //clear obstacles for when recovery started because the robot might be surrounded by obstacles
    stringstream ss; corobot_common::Goal topicMsg;
    ss.str(""); ss << "obs cleared: " << activeObstacleList.size();
    topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);
    //ROS_DEBUG("clearing ObstacleList: %u", activeObstacleList.size());

    //ROS_DEBUG("Recovery protocol triggered");
    ss.str(""); ss << "Recovery Started";
    topicMsg.name = ss.str(); recoveryPublisher.publish(topicMsg);
    recoveryPublisher.publish(topicMsg);

    //some recovery loop here in which the robot just moves around until it sees a barcode
    //
    inRecovery = true;
}

/**
 * Wander safely (until we find a barcode and reorient ourselves)
 */
Polar APF::doRecoveryNav(LaserScan &scan){
    //Polar cmd; cmd.d = 0.1; cmd.a = 0;
    Polar cmd; cmd.d = 0.1; cmd.a = 0;
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));
    for (list<Polar>::iterator it = objects.begin(); it != objects.end(); ++it) {
        Polar objWrtRobot = *it;
        if (objWrtRobot.d <= 1 && abs(objWrtRobot.a) <= 0.6) { // if there's any object within the defined window, then turn in place
            cmd.d = 0; cmd.a = 0.4;
            cmd.d = bound(cmd.d, cmdPrev.d, 0.05); // for decelerating faster
            //ROS_DEBUG("In recovery, NavVel = \t(%.2f,%.2f)",cmd.d,cmd.a);
            cmdPrev = cmd;
            return cmd;
            //corobot::SimplePose *pointWrtGlobal = convertRobotToGlobal(objWrtRobot);
            //ROS_DEBUG("Polar Object that might be added:\t(%.2f, %.2f)", objWrtRobot.d, objWrtRobot.a);
            //ROS_DEBUG("Object in global cood:\t(%.2f, %.2f)", pointWrtGlobal->x, pointWrtGlobal->y);
        }
    }
    cmd.d = bound(cmd.d, cmdPrev.d, 0.010);
    //ROS_DEBUG("In recovery, NavVel = \t(%.2f,%.2f)",cmd.d,cmd.a);
    cmdPrev = cmd;
    return cmd;
}

