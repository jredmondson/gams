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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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
#include "Drone_RK.h"

extern "C"
{
#include "drk/flight_control_api.h"
#include "drk/drk.h"
}

gams::platforms::Drone_RK::Drone_RK (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  variables::Sensors * sensors,
  variables::Platforms & platforms,
  variables::Self & self)
  : Base (&knowledge, sensors, self), airborne_ (false)
{
  platforms["drone_rk"].init_vars (knowledge, "drone_rk");

  drk_init(0);
}

gams::platforms::Drone_RK::~Drone_RK ()
{
  drk_hover (0);
  drk_land ();
  drk_exit (0);
}

void
gams::platforms::Drone_RK::operator= (const Drone_RK & rhs)
{
  if (this != &rhs)
  {
    platforms::Base * dest = static_cast <platforms::Base *> (this);
    const platforms::Base * source =
      static_cast <const platforms::Base *> (&rhs);

    *dest = *source;
  }
}

void
gams::platforms::Drone_RK::get_sensors (variables::Sensor_Names & sensors)
{
  bool needs_change (false);

  if (sensors.size () != 1)
  {
    needs_change = true;
    sensors.resize (1);
  }
  else
  {
    if (sensors[0] != "thermal")
        needs_change = true;
  }

  if (needs_change)
  {
    sensors[0] = "thermal";
  }
}


void
gams::platforms::Drone_RK::get_position (utility::Position & position)
{
  position = position_;
}

int
gams::platforms::Drone_RK::home (void)
{
  // check if home has been set
  if (self_.device.home.size () == 3)
  {
    // read the home position
    utility::Position position;
    position.from_container (self_.device.home);

    // move to home
    move (position);
  }

  return 0;
}

int
gams::platforms::Drone_RK::move (const utility::Position & position)
{
  // check if not airborne and takeoff if appropriate
  if (!airborne_)
    takeoff ();

  // move to the position
  position_ = position;

  return 0;
}
   
int
gams::platforms::Drone_RK::sense (void)
{
  // read gps
  return 0;
}
      
int
gams::platforms::Drone_RK::analyze (void)
{
  return 0;
}

int
gams::platforms::Drone_RK::takeoff (void)
{
  if (!airborne_)
  {
    drk_takeoff ();
    drk_hover (0);
    airborne_ = true;
  }

  return 0;
}
      
int
gams::platforms::Drone_RK::land (void)
{
  if (airborne_)
  {
    drk_hover (0);
    drk_land ();
    airborne_ = false;
  }

  return 0;
}

double
gams::platforms::Drone_RK::get_gps_accuracy () const
{
  return 2.0;
}
