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
 * @file test_voting.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Tests the functionality of gams::voting classes
 **/

#include "madara/knowledge/containers/Integer.h"

#include "gams/elections/ElectionPlurality.h"
#include "gams/elections/ElectionCumulative.h"

#include "gtest/gtest.h"

namespace loggers = gams::loggers;
namespace elections = gams::elections;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

TEST(TestElections, TestCumulative)
{
  knowledge::KnowledgeBase knowledge;

  containers::Integer agent0vote ("election.president.0.agent.0->Hillary", knowledge);
  containers::Integer agent1vote ("election.president.0.agent.1->Donald", knowledge);
  containers::Integer agent2vote ("election.president.0.agent.2->Bernie", knowledge);
  containers::Integer agent3vote ("election.president.0.agent.3->Hillary", knowledge);
  containers::Integer agent4vote ("election.president.0.agent.4->Cthulhu", knowledge);
  containers::Integer agent5vote ("election.president.0.agent.5->Hillary", knowledge);
  containers::Integer agent6vote ("election.president.0.agent.6->Cthulhu", knowledge);

  elections::ElectionCumulative election (
    "election.president", "agent.0", &knowledge);

  // do a self vote for agent.0
  election.vote ("Hillary", 1);

  EXPECT_EQ(*agent0vote, 1) << "Test self vote failed";
  EXPECT_TRUE(election.has_voted("agent.0")) << "Test has_voted failed";

  // do votes for other agents
  election.vote ("agent.1", "Donald", 1);
  election.vote ("agent.2", "Bernie", 1);
  election.vote ("agent.3", "Hillary", 1);
  election.vote ("agent.4", "Cthulhu", 1);
  election.vote ("agent.5", "Hillary", 1);
  election.vote ("agent.6", "Cthulhu", 1);

  EXPECT_EQ(*agent1vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent2vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent3vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent4vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent5vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent6vote, 1) << "Testing individual agent votes failed";

  elections::CandidateList leaders = election.get_leaders (2);

  EXPECT_EQ(leaders.size(), 2) << "Testing leaders size";
  EXPECT_EQ(leaders[0], "Hillary") << "Testing the first leader";
  EXPECT_EQ(leaders[1], "Cthulhu") << "Testing the second leader";
  //
  // vote twice for agent.2
  election.vote ("agent.2", "Cthulhu", 1);
  
  leaders = election.get_leaders (2);
  EXPECT_EQ(leaders.size(), 2) << "Testing leaders size";
  EXPECT_EQ(leaders[0], "Cthulhu") << "Testing the first leader";
  EXPECT_EQ(leaders[1], "Hillary") << "Testing the second leader";
}

TEST(TestElections, TestPlurality)
{
  knowledge::KnowledgeBase knowledge;

  containers::Integer agent0vote ("election.president.0.agent.0->Hillary", knowledge);
  containers::Integer agent1vote ("election.president.0.agent.1->Donald", knowledge);
  containers::Integer agent2vote ("election.president.0.agent.2->Bernie", knowledge);
  containers::Integer agent3vote ("election.president.0.agent.3->Hillary", knowledge);
  containers::Integer agent4vote ("election.president.0.agent.4->Cthulhu", knowledge);
  containers::Integer agent5vote ("election.president.0.agent.5->Hillary", knowledge);
  containers::Integer agent6vote ("election.president.0.agent.6->Cthulhu", knowledge);

  elections::ElectionPlurality election (
    "election.president", "agent.0", &knowledge);

  // do a self vote for agent.0 and try to cheat on the plurality vote
  election.vote ("Hillary", 7);

  EXPECT_EQ(*agent0vote, 7) << "Testing self vote failed";
  EXPECT_TRUE(election.has_voted("agent.0")) << "Testing has_voted";

  // do votes for other agents
  election.vote ("agent.1", "Donald", 1);
  election.vote ("agent.2", "Bernie", 1);
  election.vote ("agent.3", "Hillary", 1);
  election.vote ("agent.4", "Cthulhu", 1);
  election.vote ("agent.5", "Hillary", 1);
  election.vote ("agent.6", "Cthulhu", 1);

  EXPECT_EQ(*agent1vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent2vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent3vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent4vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent5vote, 1) << "Testing individual agent votes failed";
  EXPECT_EQ(*agent6vote, 1) << "Testing individual agent votes failed";

  std::string leader = election.get_leader ();
  EXPECT_EQ(leader, "Hillary") << "Leader == Hillary Failed";
}

int
main (int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
