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

#include "AuctionBase.h"
#include <sstream>

gams::auctions::AuctionBase::AuctionBase (const std::string & auction_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : knowledge_ (knowledge),
    auction_prefix_ (auction_prefix),
    agent_prefix_ (agent_prefix),
    round_ (0)
{
  reset_bids_pointer ();
}

gams::auctions::AuctionBase::~AuctionBase ()
{
}

void
gams::auctions::AuctionBase::add_group (groups::GroupBase * group)
{
  groups::AgentVector members;
  group->get_members (members);

  for (groups::AgentVector::iterator i = members.begin ();
    i != members.end (); ++i)
  {
    bids_.set (*i, 0.0);
  }
}

bool
gams::auctions::AuctionBase::is_member (const std::string & agent_prefix)
{
  return bids_.exists (agent_prefix);
}

madara::knowledge::KnowledgeRecord
gams::auctions::AuctionBase::get_bid (
  const std::string & id)
{
  return bids_[id];
}

void
gams::auctions::AuctionBase::set_auction_prefix (const std::string & prefix)
{
  auction_prefix_ = prefix;
  reset_bids_pointer ();
}

void
gams::auctions::AuctionBase::set_knowledge_base (
  madara::knowledge::KnowledgeBase * knowledge)
{
  knowledge_ = knowledge;
  reset_bids_pointer ();
}

void
gams::auctions::AuctionBase::sync (void)
{
  bids_.sync_keys ();
}

void
gams::auctions::AuctionBase::bid (const std::string & agent,
  const madara::knowledge::KnowledgeRecord & amount)
{
  bids_.set (agent, amount.to_double ());
}

void gams::auctions::AuctionBase::advance_round (void)
{
  ++round_;
  reset_bids_pointer ();
}

void gams::auctions::AuctionBase::reset_round (void)
{
  round_ = 0;
  reset_bids_pointer ();
}
