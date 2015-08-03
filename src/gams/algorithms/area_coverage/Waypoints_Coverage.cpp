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

#include "gams/algorithms/area_coverage/Waypoints_Coverage.h"

#include <cmath>
#include <string>
#include <vector>

#include "gams/loggers/Global_Logger.h"

using std::string;
using std::vector;

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Waypoints_Coverage_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * /*devices*/)
{
  Base_Algorithm * result (0);
  
  if (knowledge && sensors && self && args.size () >= 1)
  {
    std::vector<utility::Position> waypoints;
    bool error = false;

    for (size_t i = 0; i < args.size (); ++i)
    {
      vector <double> coords = args[i].to_doubles ();
      utility::Position w;
      if (coords.size () == 2)
      {
        w.x = coords[0];
        w.y = coords[1];
        w.z = 2;
      }
      else if (coords.size () == 3)
      {
        w.x = coords[0];
        w.y = coords[1];
        w.z = coords[2];
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::area_coverage::Waypoint_Coverage_Factory:" \
          " arg %u is of invalid size %u\n", i, coords.size ());
        error = true;
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::area_coverage::Waypoint_Coverage_Factory:" \
        " waypoint %u is \"%f,%f,%f\"\n", i, w.x, w.y, w.z);
      waypoints.push_back (w);
    }

    if (!error)
    {
      result = new area_coverage::Waypoints_Coverage (waypoints, 
        knowledge, platform, sensors, self);
    }
  }

  return result;
}

/**
 * Waypoints_Coverage is a precomputed area coverage algorithm. The agent
 * traverses the waypoints until reaching the end
 */
gams::algorithms::area_coverage::Waypoints_Coverage::Waypoints_Coverage (
  const std::vector<utility::Position>& waypoints,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Devices * devices) :
  Base_Area_Coverage (knowledge, platform, sensors, self, devices),
  waypoints_(waypoints), cur_waypoint_ (0)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::Waypoint_Coverage:" \
    " init_vars\n");

  status_.init_vars (*knowledge, "waypoints", self->id.to_integer ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::Waypoint_Coverage:" \
    " init_variable_values\n");

  status_.init_variable_values ();

  next_position_ = waypoints_[cur_waypoint_];
}

gams::algorithms::area_coverage::Waypoints_Coverage::~Waypoints_Coverage ()
{
}

void
gams::algorithms::area_coverage::Waypoints_Coverage::operator= (
  const Waypoints_Coverage& rhs)
{
  if (this != &rhs)
  {
    this->waypoints_ = rhs.waypoints_;
    this->cur_waypoint_ = rhs.cur_waypoint_;
    this->Base_Area_Coverage::operator= (rhs);
  }
}

int
gams::algorithms::area_coverage::Waypoints_Coverage::analyze ()
{
  int ret_val (OK);

  if (cur_waypoint_ >= waypoints_.size ())
  {
    ret_val = FINISHED;
    status_.finished = 1;
  }

  return ret_val;
}

/**
 * The next destination is simply the next point in the list
 */
void
gams::algorithms::area_coverage::Waypoints_Coverage::generate_new_position ()
{
  ++cur_waypoint_;
  if (cur_waypoint_ < waypoints_.size ())
    next_position_ = waypoints_[cur_waypoint_];
  else
    cur_waypoint_ = waypoints_.size (); // prevent overflow to become valid again
}
