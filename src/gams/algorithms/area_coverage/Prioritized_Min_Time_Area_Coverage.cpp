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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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
 * @file Prioritized_Min_Time_Area_Coverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Agents mark a sensor with their time when they enter a discretized cell of a 
 * region. Each agent selects a destination coordinate which provides the 
 * highest increase in sensor utility determined by time since last observation.
 **/

#include "gams/algorithms/area_coverage/Prioritized_Min_Time_Area_Coverage.h"

#include <string>
using std::string;
#include <iostream>
using std::cerr;
using std::endl;
#include <cmath>
#include <set>
using std::set;

#include "gams/utility/GPS_Position.h"
#include "gams/utility/Position.h"

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Prioritized_Min_Time_Area_Coverage_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  Base_Algorithm * result (0);
  
  if (knowledge && sensors && self && args.size () > 0)
  {
    result = new area_coverage::Prioritized_Min_Time_Area_Coverage (
      args[0] /* search area id*/,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::area_coverage::Prioritized_Min_Time_Area_Coverage::
  Prioritized_Min_Time_Area_Coverage (
  const Madara::Knowledge_Record& search_id,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self, const string& algo_name) :
  Min_Time_Area_Coverage (search_id, knowledge, platform, sensors, self, algo_name)
{
}

void
gams::algorithms::area_coverage::Prioritized_Min_Time_Area_Coverage::operator= (
  const Prioritized_Min_Time_Area_Coverage & rhs)
{
  this->Min_Time_Area_Coverage::operator= (rhs);
}

double
gams::algorithms::area_coverage::Prioritized_Min_Time_Area_Coverage::get_utility (
  const utility::Position& start, const utility::Position& end,
  set<utility::Position>& online)
{
  /**
   * check each valid position and add its value to utility if it is along
   * the possible travel path of the agent
   */
  double util = 0.0;
  const double radius =
    min_time_.get_range () / min_time_.get_discretization ();
  for (set<utility::Position>::const_iterator it = valid_positions_.begin ();
    it != valid_positions_.end (); ++it)
  {
    if (start.distance_to_2d (end, *it) < radius)
    {
      const utility::GPS_Position gps = min_time_.get_gps_from_index (*it);
      double time = min_time_.get_value (*it) * search_area_.get_priority (gps);
      double delta_util = pow (time, 3.0);
      util += delta_util;
      online.insert (*it);
    }
  }
  
  // modify the utility based on the distance that will be travelled
  util = util / sqrt (start.distance_to_2d (end) + 1);
  return util;
}
