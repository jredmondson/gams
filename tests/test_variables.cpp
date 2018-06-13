/**
 * Copyright (c) 2014-2018 Carnegie Mellon University. All Rights Reserved.
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

#include "gtest/gtest.h"

namespace transport = madara::transport;
namespace pose = gams::pose;
namespace knowledge = madara::knowledge;
namespace variables = gams::variables;

TEST(TestVariables, TestAccent)
{

}

TEST(TestVariables, TestAgent)
{
  knowledge::KnowledgeBase context;

  variables::Agent agent;
  agent.init_vars (context, "agent.tester");

  std::vector <double> position_vector = {42.214, -73.11142, 100.25};
  std::vector <double> angle_vector = {25.214, 73.829, 342.11};

  // set position-related containers
  agent.dest.set (position_vector);
  agent.home.set (position_vector);
  agent.location.set (position_vector);
  agent.source.set (position_vector); 

  // set angle-related containers
  agent.dest_orientation.set (angle_vector);
  agent.orientation.set (angle_vector);
  agent.source_orientation.set (angle_vector);

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

  EXPECT_EQ(agent.dest.to_record ().to_doubles (), position_vector)
      << "Testing Agent.dest failed.";
  EXPECT_EQ(agent.home.to_record ().to_doubles (), position_vector)
      << "Testing Agent.home failed.";
  EXPECT_EQ(agent.location.to_record ().to_doubles (), position_vector)
      << "Testing Agent.location failed.";
  EXPECT_EQ(agent.source.to_record ().to_doubles (), position_vector)
      << "Testing Agent.source failed.";
  
  EXPECT_EQ(agent.dest_orientation.to_record ().to_doubles (), angle_vector)
      << "Testing Agent.dest_orientation failed.";
  
  EXPECT_EQ(agent.orientation.to_record ().to_doubles (), angle_vector)
      << "Testing Agent.orientation failed.";
  
  EXPECT_EQ(agent.source_orientation.to_record ().to_doubles (), angle_vector)
      << "Testing Agent.source_orientation failed.";

  // test string containers
  EXPECT_EQ(agent.algorithm, "search")
      << "Testing Agent.algorithm failed.";
  
  EXPECT_EQ(agent.coverage_type, "pac")
      << "Testing Agent.coverage_type failed.";
 
  EXPECT_EQ(agent.last_algorithm, "search")
      << "Testing Agent.last_algorithm failed.";
  
  EXPECT_EQ(agent.next_coverage_type, "rec")
      << "Testing Agent.next_coverage_type failed.";
  
  EXPECT_EQ(agent.prefix, "agent.tester")
      << "Testing Agent.prefix failed.";

  // test double containers

  EXPECT_EQ(agent.desired_altitude, 800.24)
      << "Testing Agent.desired_altitude failed.";
  
  EXPECT_EQ(agent.loop_hz, 30.0)
      << "Testing Agent.loop_hz failed.";
  
  EXPECT_EQ(agent.send_hz, 5.0)
      << "Testing Agent.send_hz failed.";
  
  EXPECT_EQ(agent.temperature, 56.5)
      << "Testing Agent.temperature failed.";
  
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
  
  EXPECT_EQ(agent.algorithm_accepts, 5)
    << "Testing Agent.algorithm_accepts failed.";
  
  EXPECT_EQ(agent.algorithm_id, 3)
    << "Testing Agent.algorithm_id failed.";

  EXPECT_EQ(agent.algorithm_rejects, 12)
    << "Testing Agent.algorithm_rejects failed.";

  EXPECT_EQ(agent.battery_remaining, 82)
    << "Testing Agent.battery_remaining failed.";

  EXPECT_EQ(agent.bridge_id, 7)
    << "Testing Agent.bridge_id failed.";
  
  EXPECT_EQ(agent.gams_debug_level, 4)
    << "Testing Agent.gams_debug_level failed.";
  
  EXPECT_EQ(agent.is_mobile, 1)
    << "Testing Agent.is_mobile failed.";
  
  EXPECT_EQ(agent.last_algorithm_id, 3)
    << "Testing Agent.last_algorithm_id failed.";
  
  EXPECT_EQ(agent.madara_debug_level, 3)
    << "Testing Agent.madara_debug_level failed.";
  
  EXPECT_EQ(agent.search_area_id, 2)
    << "Testing Agent.search_area_id failed.";
  
}

TEST(TestVariables, TestSensor)
{

}

TEST(TestVariables, TestSwarm)
{
  knowledge::KnowledgeBase context;

  variables::Swarm swarm;
  swarm.init_vars (context);

  swarm.algorithm = "null";
  swarm.algorithm_id = 6;
  swarm.min_alt = 15.0;
  swarm.size = 5;

  EXPECT_EQ(swarm.algorithm, "null")
    << "Testing Swarm.algorithm failed.";

  EXPECT_EQ(swarm.algorithm_id, 6)
    << "Testing Swarm.algorithm_id failed.";

  EXPECT_EQ(swarm.min_alt, 15.0)
    << "Testing Swarm.min_alt failed.";

  EXPECT_EQ(swarm.size, 5)
    << "Testing Swarm.size failed.";
}

int
main (int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
