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

#include "madara/knowledge/containers/Double.h"

#include "gams/auctions/AuctionMaximumBid.h"
#include "gams/auctions/AuctionMinimumBid.h"
#include "gams/auctions/AuctionFactoryRepository.h"

namespace loggers = gams::loggers;
namespace auctions = gams::auctions;
namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

void test_minimum_auction (void)
{
  loggers::global_logger->log (
    loggers::LOG_ALWAYS, "Testing AuctionMinimumBid\n");

  knowledge::KnowledgeBase knowledge;

  containers::Double agent0bid ("auction.distances.0.agent.0", knowledge);
  containers::Double agent1bid ("auction.distances.0.agent.1", knowledge);
  containers::Double agent2bid ("auction.distances.0.agent.2", knowledge);
  containers::Double agent3bid ("auction.distances.0.agent.3", knowledge);

  auctions::AuctionMinimumBid auction (
    "auction.distances", "agent.0", &knowledge);

  // do a self bid for agent.0
  auction.bid (2.0);

  if (*agent0bid == 2.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing self bid: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing self bid: FAIL\n");
  }

  if (auction.get_bid ("agent.0") == 2.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing get_bid: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing get_bid: FAIL\n");
  }

  // do bids for other agents
  auction.bid ("agent.1", 7.0);
  auction.bid ("agent.2", 1.0);
  auction.bid ("agent.3", 10.0);

  if (*agent1bid == 7.0 && *agent2bid == 1.0 && *agent3bid == 10.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing all agent bids: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, " Testing all agent bids: FAIL\n");
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.0: %f\n", *agent0bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.1: %f\n", *agent1bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.2: %f\n", *agent2bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.3: %f\n", *agent3bid);
  }

  std::string leader = auction.get_leader ();

  if (leader == "agent.2")
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Leader == agent.2: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Leader == %s: FAIL\n",
      leader.c_str ());
  }
}

void test_maximum_auction (void)
{
  loggers::global_logger->log (
    loggers::LOG_ALWAYS, "Testing AuctionMaximumBid\n");

  knowledge::KnowledgeBase knowledge;

  containers::Double agent0bid ("auction.distances.0.agent.0", knowledge);
  containers::Double agent1bid ("auction.distances.0.agent.1", knowledge);
  containers::Double agent2bid ("auction.distances.0.agent.2", knowledge);
  containers::Double agent3bid ("auction.distances.0.agent.3", knowledge);

  auctions::AuctionMaximumBid auction (
    "auction.distances", "agent.0", &knowledge);

  // do a self bid for agent.0
  auction.bid (2.0);

  if (*agent0bid == 2.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing self bid: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing self bid: FAIL\n");
  }

  if (auction.get_bid ("agent.0") == 2.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing get_bid: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing get_bid: FAIL\n");
  }

  // do bids for other agents
  auction.bid ("agent.1", 7.0);
  auction.bid ("agent.2", 1.0);
  auction.bid ("agent.3", 10.0);

  if (*agent1bid == 7.0 && *agent2bid == 1.0 && *agent3bid == 10.0)
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Testing all agent bids: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, " Testing all agent bids: FAIL\n");
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.0: %f\n", *agent0bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.1: %f\n", *agent1bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.2: %f\n", *agent2bid);
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  agent.3: %f\n", *agent3bid);
  }

  std::string leader = auction.get_leader ();

  if (leader == "agent.3")
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Leader == agent.3: SUCCESS\n");
  }
  else
  {
    loggers::global_logger->log (
      loggers::LOG_ALWAYS, "  Leader == %s: FAIL\n",
      leader.c_str ());
  }
}

int
main (int, char **)
{
  test_minimum_auction ();
  test_maximum_auction ();
  return 0;
}
