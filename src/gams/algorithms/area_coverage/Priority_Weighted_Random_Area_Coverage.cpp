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
 * @file Priority_Weighted_Random_Area_Coverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized Random Area Coverage prioritizes certain regions of a search area
 * based on specified priorities
 **/

#include "gams/algorithms/area_coverage/Priority_Weighted_Random_Area_Coverage.h"

#include <iostream>
#include <vector>

using std::string;
using std::vector;
using std::cout;
using std::endl;

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Priority_Weighted_Random_Area_Coverage_Factory::create (
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
            result = new area_coverage::Priority_Weighted_Random_Area_Coverage (
              args[0].to_string () /* search area id*/,
              ACE_Time_Value (args[1].to_double ()) /* exec time */,
              knowledge, platform, sensors, self, devices);
          }
          else
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_ERROR,
               "gams::algorithms::Priority_Weighted_Random_Area_Coverage_Factory::create:" \
              " invalid second arg, expected double\n");
          }
        }
        else
        {
          result = new area_coverage::Priority_Weighted_Random_Area_Coverage (
            args[0].to_string () /* search area id*/,
            ACE_Time_Value (0.0) /* run forever */,
            knowledge, platform, sensors, self, devices);
        }
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::algorithms::Priority_Weighted_Random_Area_Coverage_Factory::create:" \
          " invalid first arg, expected string\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::Priority_Weighted_Random_Area_Coverage_Factory::create:" \
        " expected 1 or 2 args\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::Priority_Weighted_Random_Area_Coverage_Factory::create:" \
      " invalid knowledge, sensors, self, or devices parameters\n");
  }

  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::Priority_Weighted_Random_Area_Coverage_Factory::create:" \
      " unknown error creating algorithm\n");
  }

  return result;
}

/**
 * We precompute the total priority of the search area as this is a constant.
 * The total priority is calculated based on the priority and area of each 
 * individual region.
 */
gams::algorithms::area_coverage::Priority_Weighted_Random_Area_Coverage::
Priority_Weighted_Random_Area_Coverage (
  const string& search_id,
  const ACE_Time_Value& e_time,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Devices * devices) :
  Base_Area_Coverage (knowledge, platform, sensors, self, devices, e_time),
  total_priority_ (0.0)
{
  // init status vars
  status_.init_vars (*knowledge, "pwrac", self->id.to_integer ());
  status_.init_variable_values ();

  // get search area
  search_area_.from_container (*knowledge, search_id);

  // calculate total priority
  const vector<utility::Prioritized_Region>& regions =
    search_area_.get_regions ();
  for (unsigned int i = 0; i < regions.size (); ++i)
  {
    total_priority_ += regions[i].get_area () * regions[i].priority;
    priority_total_by_region_.push_back (total_priority_);
  }

  // generate first position to move
  generate_new_position ();
}

void
gams::algorithms::area_coverage::Priority_Weighted_Random_Area_Coverage::
  operator= (const Priority_Weighted_Random_Area_Coverage & rhs)
{
  if (this != &rhs)
  {
    this->search_area_ = rhs.search_area_;
    this->priority_total_by_region_ = rhs.priority_total_by_region_;
    this->total_priority_ = rhs.total_priority_;
    this->Base_Area_Coverage::operator= (rhs);
  }
}

/**
 * A new position is selected by selecting a random region based on the 
 * weighting of the region and the total search area priority. A random position
 * is then selected from that region.
 */
void
gams::algorithms::area_coverage::Priority_Weighted_Random_Area_Coverage::
  generate_new_position ()
{
  // select region
  double selected_rand = Madara::Utility::rand_double (0.0, total_priority_);
  const utility::Prioritized_Region* selected_region = 0;
  for (unsigned int i = 0; i < search_area_.get_regions ().size (); ++i)
  {
    if (priority_total_by_region_[i] > selected_rand)
    {
      selected_region = &((search_area_.get_regions ())[i]);
      break;
    }
  }

  // select point in region
  do
  {
    next_position_.latitude (Madara::Utility::rand_double (selected_region->min_lat_,
      selected_region->max_lat_));
    next_position_.longitude (Madara::Utility::rand_double (selected_region->min_lon_,
      selected_region->max_lon_));
    next_position_.altitude (Madara::Utility::rand_double (selected_region->min_alt_,
      selected_region->max_alt_));
  }
  while (!selected_region->contains (next_position_));

  // found an acceptable position, so set it as next
  utility::GPS_Position current;
  current.from_container (self_->device.location);
  next_position_.altitude (current.altitude ());
  // TODO: update when altitude is handled
}
