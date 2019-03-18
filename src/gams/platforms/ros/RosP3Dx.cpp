/**
 * Copyright (c) 2014-2017 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/

/**
 * @file RosP3Dx.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the Stage simulated Pioneer 3D-X robot class
 */

#ifdef _GAMS_ROS_ // only compile this if we are using ROS

#include "gams/platforms/ros/RosP3Dx.h"

#include <iostream>
using std::endl;
using std::cout;
using std::cerr;
using std::string;

namespace knowledge = madara::knowledge;

#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"

gams::platforms::BasePlatform *
gams::platforms::RosP3DxFactory::create (
  const knowledge::KnowledgeMap & /*args*/,
  knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors, variables::Platforms * platforms,
  variables::Self * self)
{
  BasePlatform * result (0);

  if (knowledge && sensors && platforms && self)
    result = new RosP3Dx (knowledge, sensors, platforms, self);

  return result;
}

gams::platforms::RosP3Dx::RosP3Dx (
  knowledge::KnowledgeBase * knowledge, 
  variables::Sensors * sensors,
  variables::Platforms * platforms, variables::Self * self) :
  RosBase (knowledge, sensors, self),
  ros_namespace_(knowledge->get (".ros_namespace").to_string ()), 
  node_handle_ (ros_namespace_),
  move_client_ (ros_namespace_ + std::string ("/move_base"), true), 
  spinner_ (1), init_pose_set_ (false)
{
  ROS_INFO("creating RosP3Dx");
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  // wait up to 5 sec for server to come up
  cerr << "attempting connection to " << ros_namespace_ << "/move_base" << endl;
  size_t i;
  for(i = 0; i < 5; ++i)
    if(move_client_.waitForServer(ros::Duration(1.0)))
      break;

  if(i == 5)
  {
    ROS_INFO("move_base action server did not come up within timeout period");
    exit(-1);
  }

  status_.movement_available = 1;

  // get initial position from knowledge base
  knowledge::KnowledgeRecord record = knowledge->get (".initial_position");
  std::vector <double> coords = record.to_doubles ();
  pose::Position p(get_frame (), coords[0], coords[1], coords[2]);
  set_initial_position(p);

  // probably not needed?
  self_->agent.desired_altitude = 0.05;

  // start spinner for reading position from AMCL
  spinner_.start ();

  // platform is ready
  ready_ = true;
}

std::string
gams::platforms::RosP3Dx::get_id () const
{
  return "RosP3Dx";
}

std::string
gams::platforms::RosP3Dx::get_name () const
{
  return "ROS Pioneer 3DX";
}

int
gams::platforms::RosP3Dx::move (const pose::Position & position,
        const pose::PositionBounds &/*bounds*/)
{
  // generate message
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now ();
  goal.target_pose.pose.position.x = position.x ();
  goal.target_pose.pose.position.y = position.y ();
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  // send the goal
  ROS_INFO("Sending goal");
  move_client_.sendGoal(goal);

  return 0;
}

void
gams::platforms::RosP3Dx::set_initial_position (const pose::Position& p)
{
  // send pose estimate
  ROS_INFO("Send initial pose estimate");
  geometry_msgs::PoseWithCovarianceStamped init_pose;
  init_pose.header.frame_id = "/map";
  init_pose.header.stamp = ros::Time::now ();

  // initial position and orientation
  init_pose.pose.pose.position.x = p.x ();
  init_pose.pose.pose.position.y = p.y ();
  init_pose.pose.pose.position.z = 0;
  init_pose.pose.pose.orientation.x = 0.0;
  init_pose.pose.pose.orientation.y = 0.0;
  if(p.z () == 90)
  {
    init_pose.pose.pose.orientation.z = -0.704976308536;
    init_pose.pose.pose.orientation.w = 0.709230854097;
  }
  else if(p.z () == 180 || p.z () == -180)
  {
    init_pose.pose.pose.orientation.z = -1;
    init_pose.pose.pose.orientation.w = 0;
  }
  else if(p.z () == 0)
  {
    init_pose.pose.pose.orientation.z = 0;
    init_pose.pose.pose.orientation.w = 1;
  }

  // these may need to be tuned; no idea what these do
  for(size_t i = 0; i < 35; ++i)
    init_pose.pose.covariance[i] = 0.0;
  init_pose.pose.covariance[0] = 0.25;
  init_pose.pose.covariance[7] = 0.25;
  init_pose.pose.covariance[35] = 0.06853891945200942;

  // create subscriber/publisher for acceptance
  ros::Publisher init_pose_pub =
    node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      ros_namespace_ + std::string("/initialpose"), 1);
  amcl_pose_sub_ = node_handle_.subscribe(
    ros_namespace_ + "/amcl_pose", 1, &RosP3Dx::update_pose, this);

  // attempt to send initial pose
  ros::Rate timer(5);
  size_t i;
  for(i = 0; i < 100 && !init_pose_set_; ++i)
  {
    init_pose_pub.publish(init_pose);
    ros::spinOnce ();
    timer.sleep ();
  }
  if(i == 100)
    ROS_INFO("Unable to set initial pose");
}

void
gams::platforms::RosP3Dx::update_pose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // need to mutex protect this
  init_pose_set_ = true;
  pose::Position p (get_frame ());
  p.x (msg.pose.pose.position.x);
  p.y (msg.pose.pose.position.y);
  p.z (msg.pose.pose.position.z);
  p.to_container (self_->agent.location);
}

#endif // _GAMS_ROS_

