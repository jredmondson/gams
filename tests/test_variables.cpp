/**
 * Copyright(c) 2014-2018 Carnegie Mellon University. All Rights Reserved.
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
 * @file test_variables.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Tests the functionality of gams::variables classes
 **/

#include <iostream>

#include "gams/pose/Position.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/pose/GPSFrame.h"
#include "gams/variables/Agent.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Swarm.h"

#include "gams/variables/AccentStatus.h"

namespace transport = madara::transport;
namespace pose = gams::pose;
namespace knowledge = madara::knowledge;
namespace variables = gams::variables;

int gams_fails = 0;

void
test_accent(void)
{

}

void
test_agent(void)
{
  std::cout << "Testing Agent...\n";

  knowledge::KnowledgeBase context;

  variables::Agent agent;
  agent.init_vars(context, "agent.tester");

  std::vector <double> position_vector = {42.214, -73.11142, 100.25};
  std::vector <double> angle_vector = {25.214, 73.829, 342.11};

  // set position-related containers
  agent.dest.set(position_vector);
  agent.home.set(position_vector);
  agent.location.set(position_vector);
  agent.source.set(position_vector); 

  // set angle-related containers
  agent.dest_orientation.set(angle_vector);
  agent.orientation.set(angle_vector);
  agent.source_orientation.set(angle_vector);

  // set string containers
  agent.algorithm = "search";
  agent.coverage_type = "pac";
  agent.last_algorithm = "search";
  agent.next_coverage_type = "rec";

  
  // set double containers
  agent.desired_altitude = 800.24;
  agent.loop_hz = 30.0;
  agent.send_hz = 5.0;
  agent.temperature = 56.5;

  // set integer containers
  agent.algorithm_accepts = 5;
  agent.algorithm_id = 3;
  agent.algorithm_rejects = 12;
  agent.battery_remaining = 82;
  agent.bridge_id = 7;
  agent.gams_debug_level = 4;
  agent.is_mobile = 1;
  agent.last_algorithm_id = 3;
  agent.madara_debug_level = 3;
  agent.search_area_id = 2;

  // test the double vectors

  std::cout << "  Testing Agent.dest: ";
  if (agent.dest.to_record().to_doubles() == position_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.home: ";
  if (agent.home.to_record().to_doubles() == position_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.location: ";
  if (agent.location.to_record().to_doubles() == position_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.source: ";
  if (agent.source.to_record().to_doubles() == position_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.dest_orientation: ";
  if (agent.dest_orientation.to_record().to_doubles() == angle_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.orientation: ";
  if (agent.orientation.to_record().to_doubles() == angle_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.source_orientation: ";
  if (agent.source_orientation.to_record().to_doubles() == angle_vector)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }

  // test string containers

  std::cout << "  Testing Agent.algorithm: ";
  if (agent.algorithm == "search")
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.coverage_type: ";
  if (agent.coverage_type == "pac")
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.last_algorithm: ";
  if (agent.last_algorithm == "search")
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.next_coverage_type: ";
  if (agent.next_coverage_type == "rec")
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.prefix: ";
  if (agent.prefix == "agent.tester")
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }

  // test double containers

  std::cout << "  Testing Agent.desired_altitude: ";
  if (agent.desired_altitude == 800.24)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.loop_hz: ";
  if (agent.loop_hz == 30.0)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.send_hz: ";
  if (agent.send_hz == 5.0)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.temperature: ";
  if (agent.temperature == 56.5)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  // test double containers

  agent.algorithm_accepts = 5;
  agent.algorithm_id = 3;
  agent.algorithm_rejects = 12;
  agent.battery_remaining = 82;
  agent.bridge_id = 7;
  agent.gams_debug_level = 4;
  agent.is_mobile = 1;
  agent.last_algorithm_id = 3;
  agent.madara_debug_level = 3;
  agent.search_area_id = 2;
  
  std::cout << "  Testing Agent.algorithm_accepts: ";
  if (agent.algorithm_accepts == 5)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.algorithm_id: ";
  if (agent.algorithm_id == 3)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.algorithm_rejects: ";
  if (agent.algorithm_rejects == 12)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.battery_remaining: ";
  if (agent.battery_remaining == 82)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.bridge_id: ";
  if (agent.bridge_id == 7)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.gams_debug_level: ";
  if (agent.gams_debug_level == 4)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.is_mobile: ";
  if (agent.is_mobile == 1)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.last_algorithm_id: ";
  if (agent.last_algorithm_id == 3)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.madara_debug_level: ";
  if (agent.madara_debug_level == 3)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
  
  std::cout << "  Testing Agent.search_area_id: ";
  if (agent.search_area_id == 2)
  {
    std::cout << "SUCCESS\n";
  }
  else
  {
    std::cout << "FAIL\n";
    ++gams_fails;
  }
}

void
test_sensor(void)
{

}

void
test_swarm(void)
{
  std::cout << "Testing Swarm...\n";

  knowledge::KnowledgeBase context;

  variables::Swarm swarm;
  swarm.init_vars(context);

  swarm.algorithm = "null";
  swarm.algorithm_id = 6;
  swarm.min_alt = 15.0;
  swarm.size = 5;

  std::cout << "  Testing Swarm.algorithm: " <<
   (swarm.algorithm == "null" ? "SUCCESS\n" : "FAIL\n");

  std::cout << "  Testing Swarm.algorithm_id: " <<
   (swarm.algorithm_id == 6 ? "SUCCESS\n" : "FAIL\n");

  std::cout << "  Testing Swarm.min_alt: " <<
   (swarm.min_alt == 15.0 ? "SUCCESS\n" : "FAIL\n");

  std::cout << "  Testing Swarm.size: " <<
   (swarm.size == 5 ? "SUCCESS\n" : "FAIL\n");
}

int
main(int /*argc*/, char ** /*argv*/)
{
  test_accent();
  test_agent();
  test_sensor();
  test_swarm();

  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}

