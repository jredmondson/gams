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

/**
 * @file Perimeter_Patrol.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Defines a Perimeter Patrol path for agents to follow. Used as tutorial for
 * creating custom area coverage algorithms. If this file is updated, please
 * update the wiki as well.
 */

#include "gams/loggers/Global_Logger.h"
#include "gams/algorithms/area_coverage/Perimeter_Patrol.h"

#include <vector>

#include "gams/utility/GPS_Position.h"
#include "gams/utility/Region.h"
#include "gams/utility/Search_Area.h"
#include "gams/utility/Position.h"

using std::vector;
using std::string;

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Perimeter_Patrol_Factory::create (
  const madara::Knowledge_Vector & args,
  madara::knowledge::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  Base_Algorithm * result (0);
  
  if (knowledge && sensors && self && devices)
  {
    if (args.size () >= 1)
    {
      if (args[0].is_string_type ())
      {
        if (args.size () == 2)
        {
          if (args[1].is_double_type () || args[1].is_integer_type ())
          {
            result = new area_coverage::Perimeter_Patrol (
              args[0].to_string () /* search area id*/,
              ACE_Time_Value (args[1].to_double ()) /* exec time */,
              knowledge, platform, sensors, self, devices);
          }
          else
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_ERROR,
               "gams::algorithms::Perimeter_Patrol_Factory::create:" \
              " invalid second arg, expected double\n");
          }
        }
        else
        {
          result = new area_coverage::Perimeter_Patrol (
            args[0].to_string () /* search area id*/,
            ACE_Time_Value (0.0) /* run forever */,
            knowledge, platform, sensors, self, devices);
        }
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::algorithms::Perimeter_Patrol_Factory::create:" \
          " invalid first arg, expected string\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::Perimeter_Patrol_Factory::create:" \
        " expected 1 or 2 args\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::Perimeter_Patrol_Factory::create:" \
      " invalid knowledge, sensors, self, or devices parameters\n");
  }

  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::Perimeter_Patrol_Factory::create:" \
      " unknown error creating algorithm\n");
  }

  return result;
}

/**
 * Perimeter patrol is a precomputed algorithm. The agent traverses the vertices
 * of the region in order
 */
gams::algorithms::area_coverage::Perimeter_Patrol::Perimeter_Patrol (
  const string& region_id, const ACE_Time_Value& e_time, 
  madara::knowledge::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Devices * devices) :
  Base_Area_Coverage (knowledge, platform, sensors, self, devices, e_time)
{
  // initialize some status variables
  status_.init_vars (*knowledge, "ppac", self->id.to_integer ());
  status_.init_variable_values ();

  // get waypoints
  utility::Search_Area sa;
  sa.from_container (*knowledge, region_id);
  utility::Region reg = sa.get_convex_hull ();
  vector<utility::GPS_Position> vertices = reg.vertices;

  // find closest waypoint as starting point
  size_t closest = 0;
  utility::GPS_Position current;
  current.from_container (self_->device.location);
  double min_distance = current.distance_to (vertices[0]);
  for (size_t i = 1; i < vertices.size (); ++i)
  {
    double dist = current.distance_to (vertices[i]);
    if (min_distance > dist)
    {
      dist = min_distance;
      closest = i;
    }
  }

  // add some intermediate points
  const size_t NUM_INTERMEDIATE_PTS = 5;
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    utility::GPS_Position start = vertices[(i + closest) % vertices.size ()];
    utility::GPS_Position end = vertices[(i + closest + 1) % vertices.size ()];
    double lat_dif = start.latitude () - end.latitude ();
    double lon_dif = start.longitude () - end.longitude ();

    for (size_t j = 0; j < NUM_INTERMEDIATE_PTS; ++j)
    {
      utility::GPS_Position temp (
        start.latitude () - lat_dif * j / NUM_INTERMEDIATE_PTS,
        start.longitude () - lon_dif * j / NUM_INTERMEDIATE_PTS,
        self_->device.desired_altitude.to_double ());
      waypoints_.push_back (temp);
    }
  }

  // set next_position_
  cur_waypoint_ = 0;
  next_position_ = waypoints_[cur_waypoint_];
}

gams::algorithms::area_coverage::Perimeter_Patrol::~Perimeter_Patrol ()
{
}

void
gams::algorithms::area_coverage::Perimeter_Patrol::operator= (
  const Perimeter_Patrol& rhs)
{
  if (this != &rhs)
  {
    this->Base_Area_Coverage::operator= (rhs);
    this->waypoints_ = rhs.waypoints_;
    this->cur_waypoint_ = rhs.cur_waypoint_;
  }
}

/**
 * The next destination is the next vertex in the list
 */
void
gams::algorithms::area_coverage::Perimeter_Patrol::generate_new_position ()
{
  cur_waypoint_ = (cur_waypoint_ + 1) % waypoints_.size ();
  next_position_ = waypoints_[cur_waypoint_];
}
