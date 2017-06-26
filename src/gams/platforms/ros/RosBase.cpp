/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
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

#ifdef _GAMS_ROS_ // only compile this if we are simulating in ROS

#include "gams/platforms/ros/RosBase.h"

#include <iostream>
#include <cmath>
#include <map>
#include <vector>

#include "madara/knowledge/containers/DoubleVector.h"

#include "ros/ros.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

using std::endl;
using std::cout;
using std::cerr;
using std::string;
using std::map;

gams::platforms::RosBase::RosBase (madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors, variables::Self * self) :
  BasePlatform (knowledge, sensors, self), ready_ (false)
{
  static bool init = false;
  this->frame_ = pose::gps_frame();
  //Or, for Cartesian: this->frame_ = pose::default_frame();
  if (!init)
  {
    string node_name (knowledge->get (".ros_node").to_string ());
    map<string, string> remap;
    ros::init (remap, node_name);
    init = true;
  }
}

gams::platforms::RosBase::~RosBase ()
{
}

void
gams::platforms::RosBase::operator= (const RosBase & rhs)
{
}

int
gams::platforms::RosBase::sense ()
{
  return 1;
}

int
gams::platforms::RosBase::analyze ()
{
  return 1;
}

double
gams::platforms::RosBase::get_accuracy () const
{
  return 1;
}

int
gams::platforms::RosBase::land ()
{
  return 1;
}

int
land ()
{
  return 1;
}

int
gams::platforms::RosBase::move (const pose::Position & position, const double & epsilon)
{
  return 1;
}

void
gams::platforms::RosBase::set_move_speed (const double& speed)
{
}

int
gams::platforms::RosBase::takeoff ()
{
}

void
gams::platforms::RosBase::ros_init(const std::string& n)
{
  static bool has_init = false;
  
  if(!has_init)
  {
    std::map<std::string, std::string> remap;
    ros::init(remap, n);
    has_init = true;
  }
}

void
gams::platforms::RosBase::wait_for_go () const
{
}

const gams::pose::ReferenceFrame &
gams::platforms::RosBase::get_frame (void) const
{
  return *frame_;
}

#endif // _GAMS_ROS_

