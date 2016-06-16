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
 * @file test_variables.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Tests the functionality of gams::variables classes
 **/

#include "gams/utility/Position.h"
#include "gams/utility/GPSPosition.h"
#include "gams/variables/Sensor.h"

#include "gams/variables/AccentStatus.h"

#include <string>
#include <iostream>
#include <assert.h>
#include <vector>

using gams::utility::GPSPosition;
using gams::utility::Position;
using gams::variables::Sensor;
using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace transport = madara::transport;
namespace engine = madara::knowledge;
namespace utility = madara::utility;
namespace variables = gams::variables;

void
testing_output (const string& str, const unsigned int& tabs = 0)
{
  for (unsigned int i = 0; i < tabs; ++i)
    cout << "\t";
  cout << "testing " << str << "..." << endl;
}

void
test_accent ()
{
  testing_output ("gams::variables::Accent");

  engine::KnowledgeBase knowledge;
  variables::AccentStatus cur_accent;
  cur_accent.init_vars (knowledge, "agent");

  knowledge.print ("Check the following printout for agent.accent vars.\n");
  knowledge.print ();
  
  cur_accent.init_vars (knowledge, "swarm");

  knowledge.print ("Check the following printout for swarm.accent vars.\n");
  knowledge.print ();
}

void
test_Sensor ()
{
  testing_output ("gams::variables::Sensor");

  // init knowledge base to use
  std::string host ("");
  const std::string default_multicast ("239.255.0.1:4150");
  transport::QoSTransportSettings settings;
  settings.type = transport::MULTICAST;
  if (settings.hosts.size () == 0)
  {
    settings.hosts.push_back (default_multicast);
  }
  engine::KnowledgeBase test (host, settings);

  // test constructor
  testing_output ("constructor", 1);

  /**
   * The name is used to when created a knowledge base map.
   */
  const string name ("coverage");

  /**
   * Range is the range in meters of the sensor being recorded by this object. 
   * The assumption is an omnidirectional sensor with a radius, r.
   *
   * For example, c is a discretized Position location and each half-diagonal is 
   * length r. We know that a sensor at any location in the square can sense at
   * c if the sensor range is at least r.
   *
   * ---------------
   * |\           /|
   * |  \     r /  |
   * |    \   /    |
   * |      c      |
   * |    /   \    |
   * |  /       \  |
   * |/           \|
   * ---------------
   */
  const double range = 1.0;

  /**
   * The origin is the location from which discretized positions are calculated.
   * We make the assumption that the Earth is flat over the area of operation 
   * and overlay a cartesian grid with the origin GPS location as (0,0,0).
   */
  const GPSPosition origin (40, -80); 

  // actual sensor construction
  Sensor s (name, &test, range, origin);
  assert (s.get_name () == name);
  assert (s.get_origin () == origin);
  assert (s.get_range () == range);

  // Test get/set value
  testing_output ("get/set", 1);
  double value = 1;

  /**
   * Here we test the set_value for a cartesian coordinate location. Since 
   * origin_idx defaults to (0,0,0), this is equivalent to setting the value at 
   * the origin. We use an assert call to confirm that the set is correct for
   * both cartesian and GPS location get_value function calls.
   */
  Position origin_idx;
  s.set_value (origin_idx, value);
  assert (s.get_value (origin_idx) == value);
  assert (s.get_value (origin) == value);

  /**
   * A similar set_value function call can be made using a GPS coordinates as the 
   * location of the sensor value. Again we confirm with two assert statements.
   */
  s.set_value (origin, value);
  assert (s.get_value (origin) == value);
  assert (s.get_value (origin_idx) == value);

  /**
   * Another utility functions provided by the Sensor class is the discretize()
   * function. Given a region or a convex search area it will return a 
   * set<Position> object with all valid discretized positions in the region or 
   * search area. This is used by LocalPheremoneAreaCoverage, 
   * MinTimeAreaCoverage, and some other algorithms to limit the number of 
   * positions to be checked for maximum movement utility. 
   *
   * This next section shows converting between gps coordinate and cartesian 
   * coordinate frames. Discretization is performed automatically by the Sensor 
   * class. Every GPS location is mapped to a single Position location, but 
   * every Position location maps to all the GPS locations within a square when 
   * performing a set. When calling get_gps_from_index() with a Position, it 
   * returns the GPS location of the closest discretized Position. 
   */
  GPSPosition g1 (origin);
  g1.latitude (40.01);
  value = 2;
  s.set_value (g1, value);
  Position p1 = s.get_index_from_gps (g1);
  GPSPosition g2 = s.get_gps_from_index (p1);
  assert (s.get_value (g1) == value);
  assert (s.get_value (p1) == value);
  assert (s.get_value (g2) == value);

  /**
   * While this class is coded for a group of homogeneous sensors, it could also
   * be used by a group of agents with different sensor ranges. 
   * get_discretization can be used to get the length of the sides of each 
   * discretized cell. Currently there is no functionality to set the sensor 
   * value for all cells within range of a location. 
   *
   * For further uses of Sensor, look at:
   *  - LocalPheremoneAreaCoverage
   *  - MinTimeAreaCoverage
   *  - PrioritizedMinTimeAreaCoverage
   */
}

int
main (int /*argc*/, char ** /*argv*/)
{
  test_accent ();
  test_Sensor ();
  return 0;
}
