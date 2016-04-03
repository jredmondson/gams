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
  const std::string & prefix,
  madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBidFactory:" \
    " creating auction from %s\n", prefix.c_str ());

  return new AuctionMaximumBid (prefix, knowledge);
}


gams::auctions::AuctionMaximumBid::AuctionMaximumBid (const std::string & prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : AuctionBase (prefix, knowledge)
{
  if (knowledge && prefix != "")
  {
    members_.set_name (prefix + ".members", *knowledge);
    sync ();
  }
}

gams::auctions::AuctionMaximumBid::~AuctionMaximumBid ()
{
}

void
gams::auctions::AuctionMaximumBid::add_members (const AgentVector & members)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:add_members" \
    " adding %d members\n", (int)members.size ());

  // add the members to the fast list
  fast_members_.insert (
    fast_members_.end (), members.begin (), members.end ());

  // add the members to the underlying knowledge base
  for (size_t i = 0; i < members.size (); ++i)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::auctions::AuctionMaximumBid:add_members" \
      " adding member %s to %s\n", members[i].c_str (), prefix_.c_str ());

    members_.push_back (members[i]);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MINOR,
    "gams::auctions::AuctionMaximumBid:add_members" \
    " resulting member list has %d members\n", (int)fast_members_.size ());
}

void
gams::auctions::AuctionMaximumBid::clear_members (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:clear_members" \
    " clearing all %d members\n", (int)fast_members_.size ());

  fast_members_.clear ();
  members_.resize (0);
}

void
gams::auctions::AuctionMaximumBid::get_members (AgentVector & members) const
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:get_members" \
    " retrieving member list (%d members)\n", (int)fast_members_.size ());

  members.clear ();

  members = fast_members_;
}

bool
gams::auctions::AuctionMaximumBid::is_member (const std::string & id) const
{
  AgentVector::const_iterator found =
    std::find (fast_members_.begin (), fast_members_.end (), id);

  return found != fast_members_.end ();
}

void
gams::auctions::AuctionMaximumBid::set_prefix (const std::string & prefix,
madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:set_prefix" \
    " setting prefix to %s\n", prefix.c_str ());

  AuctionBase::set_prefix (prefix, knowledge);

  if (knowledge && prefix != "")
  {
    members_.set_name (prefix + ".members", *knowledge);
    sync ();
  }
}

size_t
gams::auctions::AuctionMaximumBid::size (void)
{
  return fast_members_.size ();
}

void
gams::auctions::AuctionMaximumBid::sync (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:sync" \
    " syncing with %s source\n", prefix_.c_str ());

  members_.resize ();

  // check our size information
  size_t old_size = fast_members_.size ();
  size_t new_size = members_.size ();

  // if new size is not the same, resize fast_members
  if (old_size != new_size)
  {
    fast_members_.resize (new_size);
  }

  // iterate over the new members and update if necessary
  for (size_t i = 0; i < new_size; ++i)
  {
    /**
     * we optimize for the expected situation that any resize
     * will be growing the list and leaving earlier elements the same.
     * A string check is much faster than a string mutate.
     **/
    if (fast_members_[i] != members_[i])
    {
      fast_members_[i] = members_[i];
    }
  }
}

void
gams::auctions::AuctionMaximumBid::write (const std::string & prefix,
madara::knowledge::KnowledgeBase * knowledge) const
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::auctions::AuctionMaximumBid:write" \
    " performing initialization checks\n");

  std::string location;
  bool is_done (false);

  // set location in knowledge base to write to
  if (prefix == "")
  {
    if (prefix_ != "")
    {
      location = prefix_;
    }
    else
    {
      is_done = true;
    }
  }
  else
  {
    location = prefix;
  }

  // set knowledge base to write to
  if (!is_done && !knowledge)
  {
    if (knowledge_)
    {
      knowledge = knowledge_;
    }
    else
    {
      is_done = true;
    }
  }

  // if we have a valid location and knowledge base, continue
  if (!is_done)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::auctions::AuctionMaximumBid:write" \
      " writing auction information to %s\n", location.c_str ());

    // create members and type. Note we set type to 1.
    containers::StringVector members (location + ".members", *knowledge);
    containers::Integer type (location + ".type", *knowledge, 0);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::auctions::AuctionMaximumBid:write" \
      " writing %d members to %s.members\n",
      (int)fast_members_.size (), location.c_str ());

    members.resize ((int)fast_members_.size ());

    // iterate through the fast members list and set members at location
    for (size_t i = 0; i < fast_members_.size (); ++i)
    {
      members.set (i, fast_members_[i]);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::auctions::AuctionMaximumBid:write" \
      " no valid prefix to write to. No work to do.\n");
  }
}
