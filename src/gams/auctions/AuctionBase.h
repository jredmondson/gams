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

/**
* @file AuctionBase.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of the base auction class
**/

#ifndef   _GAMS_AUCTIONS_AUCTION_BASE_H_
#define   _GAMS_AUCTIONS_AUCTION_BASE_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/Map.h"

#include "AuctionTypesEnum.h"
#include "gams/groups/GroupBase.h"
#include "gams/groups/GroupFixedList.h"
#include "gams/GamsExport.h"

#include "gams/auctions/AuctionBid.h"

namespace gams
{
  namespace auctions
  {
    /**
    * Base class for an auction
    **/
    class GAMS_EXPORT AuctionBase
    {
    public:
      /**
       * Constructor
       * @param auction_prefix the name of the auction (e.g. auction.position)
       * @param agent_prefix   the name of this bidder (e.g. agent.0)
       * @param knowledge      the knowledge base to use for syncing
       **/
      AuctionBase (const std::string & auction_prefix = "",
        const std::string & agent_prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Constructor
      **/
      virtual ~AuctionBase ();

      /**
       * Adds a group of auction participants
       * @param  group  a group of bidders joining the auction
       **/
      virtual void add_group (groups::GroupBase * group);

      /**
       * Clears the underlying auction participants group
       **/
      virtual void clear_group (void);

      /**
      * Checks if the agent is a  member of the action participants
      * @param  agent_prefix  the participating agent's prefix (e.g. agent.0)
      * @return  true if the agent is a member of the auction
      **/
      virtual bool is_member (const std::string & agent_prefix) const;

      /**
      * Checks if the agent is a  member of the action participants
      * @param  id     the agent id (e.g. agent.0 or agent.leader). If null,
      *                uses the current agent's id
      * @return the bid of the agent. KnowledgeRecord is false/invalid if
      *         no bid has been made or if the agent is not actually an
      *         auction participant.
      **/
      virtual madara::knowledge::KnowledgeRecord get_bid (
        const std::string & id);

      /**
      * Sets the prefix for the current bidding agent
      * @param prefix   the name of the agent (e.g. agent.0)
      **/
      virtual void set_agent_prefix (const std::string & prefix);

      /**
      * Sets the prefix for the auction in the knowledge base
      * @param prefix   the name of the auction (e.g. auction.protectors)
      **/
      virtual void set_auction_prefix (const std::string & prefix);

      /**
      * Sets the knowledge base
       * @param knowledge the knowledge base to use for syncing
      **/
      virtual void set_knowledge_base (
        madara::knowledge::KnowledgeBase * knowledge);

      /**
      * Syncs the auction information from the knowledge base
      **/
      virtual void sync (void);

      /**
      * Gets the prefix for the current agent
      * @return  the name of this bidding agent (e.g. agent.0)
      **/
      const std::string & get_agent_prefix (void) const;

      /**
      * Gets the prefix for the auction in the knowledge base
      * @return  the name of the auction (e.g. auction.protectors)
      **/
      const std::string & get_auction_prefix (void) const;

      /**
      * Gets participation rate in the group for this round
      * @return  the percentage of participation (0.0-1.0)
      **/
      virtual double get_participation (void) const;

      /**
      * Bids in the auction. Uses the agent prefix that has been
      * set in this Auction as the bidder id.
      * @param  amount bidded amount. This is a very flexible amount
      *         that allows for almost any type of MADARA data type.
      *         Most auctions will probably use doubles and ints
      *         though. This does allow for exotic auctions such
      *         as parseable strings though that might be a constraint
      *         or a structured bid.
      **/
      void bid (const madara::knowledge::KnowledgeRecord & amount);

      /**
      * Bids in the auction
      * @param  agent  the agent prefix who is bidding
      * @param  amount bidded amount. This is a very flexible amount
      *         that allows for almost any type of MADARA data type.
      *         Most auctions will probably use doubles and ints
      *         though. This does allow for exotic auctions such
      *         as parseable strings though that might be a constraint
      *         or a structured bid.
      **/
      virtual void bid (const std::string & agent,
        const madara::knowledge::KnowledgeRecord & amount);

      /**
       * Returns the leader of the bidding process
       * @return the agent prefix of the leader of the auction
       **/
      virtual std::string get_leader (void) = 0;

      /**
       * Returns the list of bids in this round
       * @param bids                the map of bidders to bid amount
       * @param strip_prefix        if true, strips auction prefix from bidder 
       * @param include_all_members if true, includes empty bids from  
       **/
      virtual void get_bids (AuctionBids & bids,
        bool strip_prefix = true,
        bool include_all_members = false) const;

      /**
      * Proceeds to the next auction round in a multi-round
      * auction
      **/
      virtual void advance_round (void);

      /**
      * Retrieves the round number, usually in a multi-round auction
      * @return the agent prefix of the leader of the auction
      **/
      int get_round (void) const;

      /**
      * Resets the round
      **/
      virtual void reset_round (void);

      /**
      * Explicitly sets the round number. Should only be used by
      * advanced users
      * @param  round  the round number for the auction
      **/
      void set_round (int round);

      /**
      * Returns the full auction prefix string, including round
      * @return the actual prefix in the knowledge base corresponding to
      *         the current round of this auction
      **/
      std::string get_auction_round_prefix (void) const;

    protected:

      /**
       * calls a reset on the bids_ location in the knowledge base
       * using auction_prefix_ + "." + round_.
       **/
      void reset_bids_pointer (void);

      /**
      * The knowledge base to use as a data plane
      **/
      mutable madara::knowledge::KnowledgeBase * knowledge_;

      /**
       * the prefix for the auction
       **/
      std::string auction_prefix_;

      /**
       * self prefix of the agent
       **/
      std::string agent_prefix_;

      /**
       * the auction round in a multi-round auction
       **/
      int round_;

      /**
       * convenience class for bids
       **/
      madara::knowledge::containers::Map bids_;

      /**
       * the expected participant group
       **/
      groups::GroupFixedList group_;
    };
  }
}

#include "AuctionBase.inl"

#endif // _GAMS_AUCTIONS_AUCTION_BASE_H_
