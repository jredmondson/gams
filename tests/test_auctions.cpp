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
 * @file test_auctions.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Tests the functionality of gams::auctions classes
 **/

#include "gtest/gtest.h"
#include "madara/knowledge/containers/Double.h"

#include "gams/auctions/AuctionMaximumBid.h"
#include "gams/auctions/AuctionMinimumBid.h"
#include "gams/auctions/AuctionMinimumDistance.h"
#include "gams/auctions/AuctionFactoryRepository.h"
#include "gams/groups/GroupFixedList.h"
#include "gams/platforms/NullPlatform.h"

namespace loggers = gams::loggers;
namespace auctions = gams::auctions;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

TEST(TestAuctions, TestMinimumAuctionBid)
{
  using madara::knowledge::KnowledgeRecord;
  knowledge::KnowledgeBase knowledge;

  containers::Double agent0bid ("auction.distances.0.agent.0", knowledge);
  containers::Double agent1bid ("auction.distances.0.agent.1", knowledge);
  containers::Double agent2bid ("auction.distances.0.agent.2", knowledge);
  containers::Double agent3bid ("auction.distances.0.agent.3", knowledge);

  auctions::AuctionMinimumBid auction (
    "auction.distances", "agent.0", &knowledge);

  // do a self bid for agent.0
  auction.bid (KnowledgeRecord(2.0));

  EXPECT_FLOAT_EQ(*agent0bid, 2.0) << "Testing self bid: FAIL";

  EXPECT_TRUE(auction.get_bid("agent.0") == 2.0) << "Testing get_bid: FAIL";

  // do bids for other agents
  auction.bid ("agent.1", KnowledgeRecord(7.0));
  auction.bid ("agent.2", KnowledgeRecord(1.0));
  auction.bid ("agent.3", KnowledgeRecord(10.0));

  EXPECT_FLOAT_EQ(*agent1bid, 7.0) << "Testing agent.1: FAIL";
  EXPECT_FLOAT_EQ(*agent2bid, 1.0) << "Testing agent.2: FAIL";
  EXPECT_FLOAT_EQ(*agent3bid, 10.0) << "Testing agent.3: FAIL";

  std::string leader = auction.get_leader ();

  EXPECT_EQ(leader, "agent.2") << "Leader is not the right agent.";
}

TEST(TestAuctions, TestMaximumAuctionBid)
{
  using madara::knowledge::KnowledgeRecord;
  knowledge::KnowledgeBase knowledge;

  containers::Double agent0bid ("auction.distances.0.agent.0", knowledge);
  containers::Double agent1bid ("auction.distances.0.agent.1", knowledge);
  containers::Double agent2bid ("auction.distances.0.agent.2", knowledge);
  containers::Double agent3bid ("auction.distances.0.agent.3", knowledge);

  auctions::AuctionMaximumBid auction (
    "auction.distances", "agent.0", &knowledge);

  // do a self bid for agent.0
  auction.bid (KnowledgeRecord(2.0));

  EXPECT_FLOAT_EQ(*agent0bid, 2.0) << "Testing self bid failed.";
  EXPECT_TRUE(auction.get_bid ("agent.0") == 2.0) << "Testing get_bid failed.";
 
  // do bids for other agents
  auction.bid ("agent.1", KnowledgeRecord(7.0));
  auction.bid ("agent.2", KnowledgeRecord(1.0));
  auction.bid ("agent.3", KnowledgeRecord(10.0));

  EXPECT_FLOAT_EQ(*agent1bid, 7.0) << "Testing agent.1: FAIL";
  EXPECT_FLOAT_EQ(*agent2bid, 1.0) << "Testing agent.2: FAIL";
  EXPECT_FLOAT_EQ(*agent3bid, 10.0) << "Testing agent.3: FAIL";

  std::string leader = auction.get_leader ();

  EXPECT_EQ(leader, "agent.3") << "Leader is not the right agent.";
}

TEST(TestAuctions, DISABLED_AuctionMinimumDistance)
{
  knowledge::KnowledgeBase knowledge;

  gams::groups::GroupFixedList group;
  gams::groups::AgentVector members;
  gams::variables::Platforms platforms;
  gams::variables::Agents agents;
  gams::utility::GPSPosition position;
  gams::utility::Location target;
  gams::platforms::NullPlatform platform (&knowledge, 0, &platforms, 0);

  // add some members to the group
  members.push_back ("agent.0");
  members.push_back ("agent.1");
  members.push_back ("agent.2");
  members.push_back ("agent.3");
  members.push_back ("agent.4");
  group.add_members (members);

  // initialize agent variables so we can manipulate them directly
  gams::variables::init_vars (agents, knowledge, group);

  // setup some handy references for referring to individual bids
  containers::Double agent0bid ("auction.guard_duty.0.agent.0", knowledge);
  containers::Double agent1bid ("auction.guard_duty.0.agent.1", knowledge);
  containers::Double agent2bid ("auction.guard_duty.0.agent.2", knowledge);
  containers::Double agent3bid ("auction.guard_duty.0.agent.3", knowledge);
  containers::Double agent4bid ("auction.guard_duty.0.agent.4", knowledge);

  // create some GPS locations
  position.latitude (42.0600);
  position.longitude (-72.0600);
  position.to_container (agents[0].location);

  position.latitude (42.0700);
  position.longitude (-72.0700);
  position.to_container (agents[1].location);

  position.latitude (42.0800);
  position.longitude (-72.0800);
  position.to_container (agents[2].location);

  position.latitude (42.0900);
  position.longitude (-72.0900);
  position.to_container (agents[3].location);

  position.latitude (42.1000);
  position.longitude (-72.1000);
  position.to_container (agents[4].location);

  auctions::AuctionMinimumDistance auction (
    "auction.guard_duty", "agent.0", &knowledge, &platform);

  auction.add_group (&group);

  // set the target position

  auction.set_target (position);
  auction.calculate_bids ();

  std::string closest = auction.get_leader ();

  EXPECT_EQ(closest, "agent.4") << "Wrong leader selected.";
}

int
main (int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
