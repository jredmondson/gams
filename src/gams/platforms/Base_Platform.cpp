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

#include <algorithm>
#include <cmath>

#include "Base_Platform.h"
#include "gams/utility/GPS_Position.h"

namespace platforms = gams::platforms;
namespace variables = gams::variables;

gams::platforms::Base::Base (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Self * self)
  : knowledge_ (knowledge), self_ (self), sensors_ (sensors)
{
}

gams::platforms::Base::~Base ()
{
}

void
gams::platforms::Base::operator= (const Base & rhs)
{
  if (this != &rhs)
  {
    this->knowledge_ = rhs.knowledge_;
    this->sensors_ = rhs.sensors_;
    this->status_ = rhs.status_;
    this->self_ = rhs.self_;
  }
}

gams::utility::Position *
gams::platforms::Base::get_position ()
{
  utility::Position * position = new utility::Position ();
  position->from_container (self_->device.location);
  return position;
}

double
gams::platforms::Base::get_min_sensor_range () const
{
  double min_range = DBL_MAX;
  for (variables::Sensors::const_iterator it = sensors_->begin ();
    it != sensors_->end (); ++it)
  {
    /**
     * std::min appears to be confusing Visual Studio 2010. Consequently,
     * use a ternary operator here which should be highly optimized during
     * compilation
     **/
    double cur_range (it->second->get_range ());
    min_range = cur_range < min_range ? cur_range : min_range;
  }
  return min_range;
}

double
gams::platforms::Base::get_move_speed () const
{
  return move_speed_;
}

const gams::variables::Sensor&
gams::platforms::Base::get_sensor (const std::string& name) const
{
  return *((*sensors_)[name]);
}

void
gams::platforms::Base::get_sensor_names (variables::Sensor_Names& sensors) const
{
  variables::Sensor_Names ret_val;
  for (variables::Sensors::iterator it = sensors_->begin ();
    it != sensors_->end (); ++it)
  {
    ret_val.push_back (it->first);
  }
  sensors.swap (ret_val);
}

int
gams::platforms::Base::home (void)
{
  // check if home has been set
  if (self_->device.home.size () == 3)
  {
    // read the home position
    utility::GPS_Position position;
    position.from_container (self_->device.home);

    // move to home
    move (position);
  }

  return 0;
}

int
gams::platforms::Base::move (const utility::Position & target,
  const double & epsilon)
{
  int result = 0;

  utility::GPS_Position current;
  current.from_container (self_->device.location);

  /**
   * if we are not paused, we are not already at the target,
   * and we are either not moving or the target is different
   * from the existing move location, then set status to
   * moving and return 1 (moving to the new location)
   **/
  if (!*status_.paused_moving && target != current &&
     (!*status_.moving || target != self_->device.dest))
  {
    self_->device.source = self_->device.location;
    target.to_container (self_->device.dest);

    result = 1;
    status_.moving = 1;
  }
  /**
   * otherwise, if we are approximately at the target location,
   * change status and paused to 0 and return 0 (not moving)
   **/
  else if (target.approximately_equal (current, epsilon))
  {
    status_.moving = 0;
    status_.paused_moving = 0;
    result = 0;
  }

  return result;
}


int
gams::platforms::Base::rotate (const utility::Axes &)
{
  return 0;
}

int
gams::platforms::Base::takeoff (void)
{
  return 2;
}

int
gams::platforms::Base::land (void)
{
  return 2;
}

double gams::platforms::Base::get_accuracy (void) const
{
  return 0.0;
}

void
gams::platforms::Base::pause_move (void)
{
  if (*status_.moving)
    status_.moving = 0;

  status_.paused_moving = 1;
}

void
gams::platforms::Base::set_knowledge (
  Madara::Knowledge_Engine::Knowledge_Base * rhs)
{
  knowledge_ = rhs;
}

void
gams::platforms::Base::set_move_speed (const double& speed)
{
  move_speed_ = speed;
}

void
gams::platforms::Base::set_sensors (variables::Sensors * sensors)
{
  sensors_ = sensors;
}

void
gams::platforms::Base::stop_move (void)
{
  if (*status_.moving)
    status_.moving = 0;
  status_.paused_moving = 0;

  // set source and dest to current position
  self_->device.source = self_->device.location;
  self_->device.dest = self_->device.location;
}

Madara::Knowledge_Engine::Knowledge_Base *
gams::platforms::Base::get_knowledge_base (void)
{
  return knowledge_;
}

variables::Self *
gams::platforms::Base::get_self (void)
{
  return self_;
}

variables::Sensors *
gams::platforms::Base::get_sensors (void)
{
  return sensors_;
}

variables::Platform *
gams::platforms::Base::get_platform_status (void)
{
  return &status_;
}
