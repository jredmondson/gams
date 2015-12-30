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

#include "gams/algorithms/area_coverage/UniformRandomEdgeCoverage.h"
using std::string;

#include "gams/utility/ArgumentParser.h"

gams::algorithms::BaseAlgorithm *
gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create (
  const madara::knowledge::KnowledgeMap & map,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && self)
  {
    // Use a dumb workaround for now; TODO: convert this algo to use the map
    using namespace madara::knowledge;
    KnowledgeVector args(utility::kmap2kvec(map));

    if (args.size () >= 1)
    {
      if (args[0].is_string_type ())
      {
        if (args.size () == 2)
        {
          if (args[1].is_double_type () || args[1].is_integer_type ())
          {
            result = new area_coverage::UniformRandomEdgeCoverage (
              args[0].to_string () /* search area id*/,
              ACE_Time_Value (args[1].to_double ()) /* exec time */,
              knowledge, platform, sensors, self);
          }
          else
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_ERROR,
               "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
              " invalid second arg, expected double\n");
          }
        }
        else
        {
          result = new area_coverage::UniformRandomEdgeCoverage (
            args[0].to_string () /* search area id*/,
            ACE_Time_Value (0.0) /* run forever */,
            knowledge, platform, sensors, self);
        }
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
          " invalid first arg, expected string\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
        " expected 1 or 2 args\n");
    }
  }
  else
  {
    if (knowledge)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
        " invalid knowledge parameter\n");
    }
    if (sensors)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
        " invalid sensors parameter\n");
    }
    if (self)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
        " invalid self parameter\n");
    }
  }

  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::area_coverage::UniformRandomEdgeCoverageFactory::create:" \
      " error creating algorithm\n");
  }

  return result;
}

gams::algorithms::area_coverage::
UniformRandomEdgeCoverage::UniformRandomEdgeCoverage (
  const string& prefix,
  const ACE_Time_Value& e_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents) :
  BaseAreaCoverage (knowledge, platform, sensors, self, agents, e_time)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " entered constructor\n");

  // init status vars
  status_.init_vars (*knowledge, "urec", self->id.to_integer ());
  status_.init_variable_values ();

  // generate search region
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " parsing SearchArea \"%s\"\n", prefix.c_str ());
  utility::SearchArea search;
  search.from_container (*knowledge, prefix);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " SearchArea \"%s\" is \"%s\"\n", prefix.c_str (), search.to_string ().c_str ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " getting convex hull of \"%s\"\n", prefix.c_str ());
  
  region_ = search.get_convex_hull ();

  // generate initial waypoint
  generate_new_position ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::UniformRandomEdgeCoverage::constructor:" \
    " finished constructing algorithm\n");
}

gams::algorithms::area_coverage::UniformRandomEdgeCoverage::~UniformRandomEdgeCoverage ()
{
}

void
gams::algorithms::area_coverage::UniformRandomEdgeCoverage::operator= (
  const UniformRandomEdgeCoverage & rhs)
{
  if (this != &rhs)
  {
    this->region_ = rhs.region_;
    this->BaseAreaCoverage::operator= (rhs);
  }
}

/**
 * An edge is selected at uniform random and then a point on that edge is
 * selected at uniform random.
 */
void
gams::algorithms::area_coverage::UniformRandomEdgeCoverage::generate_new_position ()
{
  // select new edge
  int num_edges = int (region_.vertices.size());
  int target_edge = madara::utility::rand_int (0, num_edges-1);

  // get endpoints
  const utility::GPSPosition & pos_1 = region_.vertices[target_edge];
  const utility::GPSPosition & pos_2 = 
    region_.vertices[(target_edge + 1) % num_edges];

  // get random point on line
  double delta_lat = pos_2.latitude () - pos_1.latitude ();
  double delta_lon = pos_2.longitude () - pos_1.longitude ();
  if (delta_lon == 0) // north/south line
  {
    const double & min = pos_1.latitude () < pos_2.latitude () ? pos_1.latitude () : pos_2.latitude ();
    const double & max = pos_1.latitude () > pos_2.latitude () ? pos_1.latitude () : pos_2.latitude ();
    next_position_.latitude (madara::utility::rand_double(min, max));
    next_position_.longitude (pos_1.longitude ());
  }
  else if (delta_lat == 0) // east/west line
  {
    const double & min = pos_1.longitude () < pos_2.longitude () ? pos_1.longitude () : pos_2.longitude ();
    const double & max = pos_1.longitude () > pos_2.longitude () ? pos_1.longitude () : pos_2.longitude ();
    next_position_.longitude (madara::utility::rand_double(min, max));
    next_position_.latitude (pos_1.latitude ());
  }
  else // other arbitrary line
  {
    const double slope = delta_lon / delta_lat;
    next_position_.latitude (madara::utility::rand_double(pos_1.latitude (), pos_2.latitude ()));
    next_position_.longitude (pos_1.longitude () + slope * (next_position_.latitude () - pos_1.latitude ()));
  }

  // fill in altitude on waypoint
  next_position_.altitude (self_->agent.desired_altitude.to_double ());
}
