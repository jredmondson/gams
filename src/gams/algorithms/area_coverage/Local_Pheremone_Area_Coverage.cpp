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
 * @file Local_Pheremone_Area_Coverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Agents mark a sensor with their time when they enter a discretized cell of a 
 * region. At each time step, they select the neighboring cell with the lowest 
 * pheremone reading as their next destination.
 **/

#include "gams/algorithms/area_coverage/Local_Pheremone_Area_Coverage.h"

#include "madara/utility/Utility.h"

#include <iostream>
using std::cerr;
using std::endl;
#include <vector>
using std::vector;

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Local_Pheremone_Area_Coverage_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
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
            result = new area_coverage::Local_Pheremone_Area_Coverage (
              args[0].to_string () /* search area id*/,
              ACE_Time_Value (args[1].to_double ()) /* exec time */,
              knowledge, platform, sensors, self, devices);
          }
          else
          {
            GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
              DLINFO "gams::algorithms::Local_Pheremone_Area_Coverage_Factory::create:" \
              " invalid second arg, expected double\n"));
          }
        }
        else
        {
          result = new area_coverage::Local_Pheremone_Area_Coverage (
            args[0].to_string () /* search area id*/,
            ACE_Time_Value (0.0) /* run forever */,
            knowledge, platform, sensors, self, devices);
        }
      }
      else
      {
        GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
          DLINFO "gams::algorithms::Local_Pheremone_Area_Coverage_Factory::create:" \
          " invalid first arg, expected string\n"));
      }
    }
    else
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::algorithms::Local_Pheremone_Area_Coverage_Factory::create:" \
        " expected 1 or 2 args\n"));
    }
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::algorithms::Local_Pheremone_Area_Coverage_Factory::create:" \
      " invalid knowledge, sensors, self, or devices parameters\n"));
  }

  if (result == 0)
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::algorithms::Local_Pheremone_Area_Coverage_Factory::create:" \
      " unknown error creating algorithm\n"));
  }

  return result;
}

gams::algorithms::area_coverage::Local_Pheremone_Area_Coverage::
Local_Pheremone_Area_Coverage (
  const std::string& search_id,
  const ACE_Time_Value& e_time,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Devices * devices) :
  Base_Area_Coverage (knowledge, platform, sensors, self, devices, e_time),
  search_area_ (
    utility::parse_search_area (*knowledge, search_id)),
  pheremone_ (search_id + ".pheremone", knowledge)
{
  // init status vars
  status_.init_vars (*knowledge, "lpac");

  // fill out pheremone sensor
  /**
   * See Min_Time_Area_Coverage.cpp for why this needs to be updated later
   */
  utility::GPS_Position origin;
  Madara::Knowledge_Engine::Containers::Native_Double_Array origin_container;
  origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
  origin.from_container (origin_container);
  pheremone_.set_origin (origin);
  pheremone_.set_range (5.0);
  
  // generate first position to move
  generate_new_position ();
}

void
gams::algorithms::area_coverage::Local_Pheremone_Area_Coverage::operator= (
  const Local_Pheremone_Area_Coverage & rhs)
{
  if (this != &rhs)
  {
    this->search_area_ = rhs.search_area_;
    this->pheremone_ = rhs.pheremone_;
    this->Base_Area_Coverage::operator= (rhs);
  }
}

void
gams::algorithms::area_coverage::Local_Pheremone_Area_Coverage::
  generate_new_position ()
{
  // get current location
  utility::GPS_Position cur_gps;
  cur_gps.from_container (self_->device.location);

  // create possible next positions
  const int num_possible = 12;
  utility::Position possible[num_possible];
  utility::Position cur = pheremone_.get_index_from_gps (cur_gps);
  vector<unsigned int> selection;
  for (unsigned int i = 0; i < num_possible; ++i)
  {
    possible[i] = cur;
    selection.push_back (i);
  }
  ++possible[0].x; ++possible[0].y;
  ++possible[1].x;
  ++possible[2].x; --possible[2].y;
  --possible[3].y;
  --possible[4].x; --possible[4].y;
  --possible[5].x;
  --possible[6].x; ++possible[6].y;
  ++possible[7].y;

  /**
   * We consider the possibility that the agent drifts outside the actual area
   * of operation. A more robust way to do this would be to find the closest 
   * cell in the area and go to that, however, this is simpler and has not
   * failed in simulation. Real-world experiments are likely to differ and may 
   * require the more robust solution. 
   */
  possible[8].x += 2;
  possible[9].y += 2;
  possible[10].x -= 2;
  possible[11].y -= 2;

  // find lowest pheremone concentration of possible coords in search_area
  utility::GPS_Position lowest = cur_gps;
  utility::Position s;
  double concentration = DBL_MAX;
  std::random_shuffle (selection.begin (), selection.end ());
  for (unsigned int i = 0; i < num_possible; ++i)
  {
    const int index = selection[i]; // get randomized index

    // update executions value if necessary
    const double my_concentration = pheremone_.get_value (possible[index]);
    if (my_concentration > executions_)
      executions_ = my_concentration;

    // check if new min found and update if necessary
    if (concentration > my_concentration)
    {
      utility::GPS_Position possible_gps =
        pheremone_.get_gps_from_index (possible[index]);
      if (search_area_.contains (possible_gps))
      {
        {
          concentration = my_concentration;
          lowest = possible_gps;
          s = possible[index];
        }
      }
    }
  }

  // update pheremone value
  pheremone_.set_value (lowest, executions_ + 1);

  // assign new next
  // TODO: fix with proper altitude
  lowest.altitude (self_->device.desired_altitude.to_double ());
  next_position_ = lowest;
}
