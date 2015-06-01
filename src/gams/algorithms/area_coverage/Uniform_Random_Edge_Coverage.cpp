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

#include "gams/algorithms/area_coverage/Uniform_Random_Edge_Coverage.h"

gams::algorithms::Base_Algorithm *
gams::algorithms::area_coverage::Uniform_Random_Edge_Coverage_Factory::create (
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
    result = new area_coverage::Uniform_Random_Edge_Coverage (
      args[0] /* region id*/,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::area_coverage::
Uniform_Random_Edge_Coverage::Uniform_Random_Edge_Coverage (
  const Madara::Knowledge_Record& prefix,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  Base_Area_Coverage (knowledge, platform, sensors, self)
{
  // init status vars
  status_.init_vars (*knowledge, "urec");

  // generate search region
  utility::Search_Area search = utility::parse_search_area (
    *knowledge, prefix.to_string ());
  region_ = search.get_convex_hull ();

  // generate initial waypoint
  generate_new_position ();
}

gams::algorithms::area_coverage::Uniform_Random_Edge_Coverage::~Uniform_Random_Edge_Coverage ()
{
}

void
gams::algorithms::area_coverage::Uniform_Random_Edge_Coverage::operator= (
  const Uniform_Random_Edge_Coverage & rhs)
{
  if (this != &rhs)
  {
    this->region_ = rhs.region_;
    this->Base_Area_Coverage::operator= (rhs);
  }
}

/**
 * An edge is selected at uniform random and then a point on that edge is
 * selected at uniform random.
 */
void
gams::algorithms::area_coverage::Uniform_Random_Edge_Coverage::generate_new_position ()
{
  // select new edge
  int num_edges = (int)region_.vertices.size();
  int target_edge = rand() % num_edges;

  // get endpoints
  const utility::GPS_Position & pos_1 = region_.vertices[target_edge];
  const utility::GPS_Position & pos_2 = 
    region_.vertices[(target_edge + 1) % num_edges];

  // get random point on line
  double delta_lat = pos_2.latitude () - pos_1.latitude ();
  double delta_lon = pos_2.longitude () - pos_1.longitude ();
  if (delta_lon == 0) // north/south line
  {
    const double & min = pos_1.latitude () < pos_2.latitude () ? pos_1.latitude () : pos_2.latitude ();
    const double & max = pos_1.latitude () > pos_2.latitude () ? pos_1.latitude () : pos_2.latitude ();
    next_position_.latitude (Madara::Utility::rand_double(min, max));
    next_position_.longitude (pos_1.longitude ());
  }
  else if (delta_lat == 0) // east/west line
  {
    const double & min = pos_1.longitude () < pos_2.longitude () ? pos_1.longitude () : pos_2.longitude ();
    const double & max = pos_1.longitude () > pos_2.longitude () ? pos_1.longitude () : pos_2.longitude ();
    next_position_.longitude (Madara::Utility::rand_double(min, max));
    next_position_.latitude (pos_1.latitude ());
  }
  else // other arbitrary line
  {
    const double slope = delta_lon / delta_lat;
    next_position_.latitude (Madara::Utility::rand_double(pos_1.latitude (), pos_2.latitude ()));
    next_position_.longitude (pos_1.longitude () + slope * (next_position_.latitude () - pos_1.latitude ()));
  }

  // fill in altitude on waypoint
  next_position_.altitude (self_->device.desired_altitude.to_double ());
}
