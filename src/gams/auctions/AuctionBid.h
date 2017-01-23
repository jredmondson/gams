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
* @file AuctionBid.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of an auction bid instance
**/

#ifndef   _GAMS_AUCTIONS_AUCTION_BID_H_
#define   _GAMS_AUCTIONS_AUCTION_BID_H_

#include <string>
#include <vector>

#include "madara/knowledge/KnowledgeRecord.h"

#include "gams/GAMSExport.h"

namespace gams
{
  namespace auctions
  {
    /**
    * Auction bidding instance
    **/
    class GAMSExport AuctionBid
    {
    public:

      /**
       * Compares for bid less than
       **/
      bool operator< (AuctionBid & rhs);

      /**
      * Compares for bid equality
      **/
      bool operator== (AuctionBid & rhs);

      /**
      * Compares for bid greater than
      **/
      bool operator> (AuctionBid & rhs);

      /// the id of the bidder (e.g., "agent.0")
      std::string bidder;

      /// the amount of the bid (e.g., 2.50)
      madara::knowledge::KnowledgeRecord amount;
    };

    /**
     * Comparator for sorting bids in ascending order (least to greatest)
     **/
    class GAMSExport AuctionBidAscending
    {
    public:
      /**
      * Comparison for sorting
      * @param  lhs  left hand side
      * @param  rhs  right hand side
      **/
      bool operator() (AuctionBid & lhs, AuctionBid &rhs);
    };

    /**
    * Comparator for sorting bids in descending order (greater to least)
    **/
    class GAMSExport AuctionBidDescending
    {
    public:
      /**
       * Comparison for sorting
       * @param  lhs  left hand side
       * @param  rhs  right hand side
       **/
      bool operator() (AuctionBid & lhs, AuctionBid &rhs);
    };

    /// convenience typedef for vector fo AuctionBid
    typedef std::vector <AuctionBid>   AuctionBids;

    /**
     * Strips bids' bidder of an auction prefix for convenience (unsafe).
     * This is highly optimized for common usage where the prefix is obtained
     * using get_auction_round_prefix and the prefix is assumed to exist in
     * each auction bidder. This should never be called twice on the same bids,
     * and if you are calling this on the wrong prefix, you may get weird
     * results. Essentially, this is using std::string::erase with 
     * 0-prefix.size().
     * @param  prefix  the result of calling get_auction_round_prefix on
     *                 AuctionBase
     * @param  bids    the bids to modify
     **/
    void strip_prefix_fast (const std::string & prefix, AuctionBids & bids);

    /**
    * Strips bids' bidder of an auction prefix for convenience. This method
    * checks for the existence of the prefix on the bidder string before
    * trying to strip it.
    * @param  prefix  the result of calling get_auction_round_prefix on
    *                 AuctionBase
    * @param  bids    the bids to modify
    **/
    void strip_prefix_safe (const std::string & prefix, AuctionBids & bids);

    /**
     * Convenience function for sorting bids in ascending order, using
     * the AuctionBidAscending comparator
     * @param  bids  the bids to sort
     **/
    void sort_ascending (AuctionBids & bids);

    /**
    * Convenience function for sorting bids in descending order, using
    * the AuctionBidDescending comparator
    * @param  bids  the bids to sort
    **/
    void sort_descending (AuctionBids & bids);

  } // end namespace auctions
} // end namespace gams

#include "AuctionBid.inl"

#endif // _GAMS_AUCTIONS_AUCTION_BID_H_
