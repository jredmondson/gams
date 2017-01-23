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
* @file AuctionBid.inl
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the inline functions for an auction bid instance
**/

#ifndef   _GAMS_AUCTIONS_AUCTION_BID_INL_
#define   _GAMS_AUCTIONS_AUCTION_BID_INL_

#include "AuctionBid.h"

#include "madara/utility/Utility.h"

inline bool
gams::auctions::AuctionBid::operator< (AuctionBid & rhs)
{
  return amount < rhs.amount;
}

inline bool
gams::auctions::AuctionBid::operator== (AuctionBid & rhs)
{
  return amount == rhs.amount;
}

inline bool
gams::auctions::AuctionBid::operator> (AuctionBid & rhs)
{
  return amount > rhs.amount;
}

inline bool
gams::auctions::AuctionBidAscending::operator() (
  AuctionBid & lhs, AuctionBid &rhs)
{
  return lhs < rhs;
}

inline bool
gams::auctions::AuctionBidDescending::operator() (
  AuctionBid & lhs, AuctionBid &rhs)
{
  return rhs < lhs;
}

inline void
gams::auctions::sort_descending (AuctionBids & bids)
{
  std::sort (bids.begin (), bids.end (), AuctionBidDescending ());
}

inline void
gams::auctions::sort_ascending (AuctionBids & bids)
{
  std::sort (bids.begin (), bids.end (), AuctionBidAscending ());
}

inline void
gams::auctions::strip_prefix_fast (
  const std::string & prefix, AuctionBids & bids)
{
  for (size_t i = 0; i < bids.size (); ++i)
  {
    bids[i].bidder.erase (0, prefix.size ());
  }
}

inline void
gams::auctions::strip_prefix_safe (
const std::string & prefix, AuctionBids & bids)
{
  for (size_t i = 0; i < bids.size (); ++i)
  {
    if (madara::utility::begins_with (bids[i].bidder, prefix))
    {
      bids[i].bidder.erase (0, prefix.size ());
    }
  }
}

#endif // _GAMS_AUCTIONS_AUCTION_BID_H_
