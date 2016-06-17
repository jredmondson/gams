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

#include "AuctionMaximumBid.h"
#include "gams/loggers/GlobalLogger.h"
#include "madara/knowledge/containers/Integer.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

gams::auctions::AuctionMaximumBidFactory::AuctionMaximumBidFactory ()
{
}

gams::auctions::AuctionMaximumBidFactory::~AuctionMaximumBidFactory ()
{
}

gams::auctions::AuctionBase *
gams::auctions::AuctionMaximumBidFactory::create (
  const std::string & auction_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBidFactory:" \
    " creating auction from %s\n", auction_prefix.c_str ());

  return new AuctionMaximumBid (auction_prefix, agent_prefix, knowledge);
}

gams::auctions::AuctionMaximumBid::AuctionMaximumBid (
  const std::string & auction_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : AuctionBase (auction_prefix, agent_prefix, knowledge)
{
}

/**
* Constructor
**/
gams::auctions::AuctionMaximumBid::~AuctionMaximumBid ()
{

}

std::string
gams::auctions::AuctionMaximumBid::get_leader (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:" \
    " getting leader from %s\n", auction_prefix_.c_str ());

  std::string leader;

  if (knowledge_)
  {
    madara::knowledge::ContextGuard guard (*knowledge_);

    std::vector <std::string> keys;
    bids_.keys (keys);

    double leader_bid = -1;

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::auctions::AuctionMaximumBid:" \
      " iterating through bids from %s\n",
      auction_prefix_.c_str ());

    for (size_t i = 0; i < keys.size (); ++i)
    {
      double current_bid = bids_[keys[i]].to_double ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::auctions::AuctionMaximumBid:" \
        " %s: bid from %s is %f\n",
        auction_prefix_.c_str (), keys[i].c_str (), current_bid);

      if (current_bid > leader_bid)
      {
        leader_bid = current_bid;
        leader = keys[i];

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
          "gams::auctions::AuctionMaximumBid:" \
          " %s: %s is new leader of auction\n",
          auction_prefix_.c_str (), keys[i].c_str ());
      }
    }
  }

  return leader;
}