/**
* Copyright (c) 2016 Carnegie Mellon University. All Rights Reserved.
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

#include <algorithm>

#include "madara/knowledge/containers/Integer.h"

#include "AuctionMinimumDistance.h"
#include "gams/loggers/GlobalLogger.h"
#include "gams/variables/Agent.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

gams::auctions::AuctionMinimumDistanceFactory::AuctionMinimumDistanceFactory ()
{
}

gams::auctions::AuctionMinimumDistanceFactory::~AuctionMinimumDistanceFactory ()
{
}

gams::auctions::AuctionBase *
gams::auctions::AuctionMinimumDistanceFactory::create (
const std::string & auction_prefix,
const std::string & agent_prefix,
madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMinimumDistanceFactory::create:" \
    " creating auction from %s\n", auction_prefix.c_str ());

  return new AuctionMinimumDistance (auction_prefix, agent_prefix, knowledge);
}

gams::auctions::AuctionMinimumDistance::AuctionMinimumDistance (
  const std::string & auction_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform)
  : AuctionBase (auction_prefix, agent_prefix, knowledge),
    platform_ (platform)
{
}

gams::auctions::AuctionMinimumDistance::~AuctionMinimumDistance ()
{

}

std::string
gams::auctions::AuctionMinimumDistance::get_leader (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMinimumDistance::get_leader:" \
    " getting leader from %s\n", auction_prefix_.c_str ());

  std::string leader;

  AuctionBids bids;
  get_bids (bids);
  sort_ascending (bids);

  if (bids.size () > 0)
  {
    leader = bids[0].bidder;
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMinimumDistance::get_leader:" \
    " final leader is %s\n", leader.c_str ());

  return leader;
}

void gams::auctions::AuctionMinimumDistance::calculate_bids (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMinimumDistance::calculate_bids:" \
    " calculating distances as bids.\n");

  if (knowledge_ && platform_)
  {
    // initialize the agent list within the group
    variables::Agents agents;
    variables::init_vars (agents, *knowledge_, group_);

    for (size_t i = 0; i < agents.size (); ++i)
    {
      // import the agents's location into the GAMS Pose system
      gams::utility::Location location (platform_->get_frame ());
      location.from_container (agents[i].location);

      double distance = location.distance_to (target_);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::auctions::AuctionMinimumDistance::calculate_bids:" \
        " agent %s distance is %f. Bidding distance.\n",
        agents[i].prefix.c_str (), distance);

      // bid for the agent using their distance to the target
      bids_.set (agents[i].prefix, distance);
    }
  }
  else
  {
    if (!knowledge_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::auctions::AuctionMinimumDistance::calculate_bids:" \
        " ERROR: invalid pointer: knowledge_ is a null pointer." \
        " We cannot perform this auction without bidding knowledge.\n");
    }
    if (!platform_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::auctions::AuctionMinimumDistance::calculate_bids:" \
        " ERROR: invalid pointer: platform_ is a null pointer." \
        " We cannot perform this auction without a valid pose frame.\n");
    }
  }
}

void
gams::auctions::AuctionMinimumDistance::set_target (
  utility::GPSPosition target)
{
  if (platform_)
  {
    target_ = utility::Location (
      platform_->get_frame (),
      target.longitude (), target.latitude (), target.altitude ());
  }
}

void
gams::auctions::AuctionMinimumDistance::set_target (
utility::Location target)
{
  target_ = target;
}
