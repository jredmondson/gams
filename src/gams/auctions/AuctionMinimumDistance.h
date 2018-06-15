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
* @file AuctionMinimumDistance.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of the minimum distance to target auction
**/

#ifndef   _GAMS_AUCTIONS_MINIMUM_DISTANCE_H_
#define   _GAMS_AUCTIONS_MINIMUM_DISTANCE_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/containers/StringVector.h"

#include "AuctionBase.h"
#include "AuctionFactory.h"

#include "gams/platforms/BasePlatform.h"
#include "gams/groups/GroupFixedList.h"
#include "gams/pose/Position.h"

namespace gams
{
  namespace auctions
  {
    /**
    * An auction where the winner is the closest agent to a location
    **/
    class GAMS_EXPORT AuctionMinimumDistance : public AuctionBase
    {
    public:
      /**
      * Constructor.
      * @param auction_prefix the name of the auction (e.g. auction.position)
      * @param agent_prefix   the name of this bidder (e.g. agent.0)
      * @param knowledge      the knowledge base to use for syncing
      * @param platform       the platform that contains a frame of reference
      **/
      AuctionMinimumDistance (const std::string & auction_prefix = "",
        const std::string & agent_prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0);

      /**
      * Constructor
      **/
      virtual ~AuctionMinimumDistance ();

      /**
      * Returns the leader of the bidding process
      * @return the agent prefix of the leader of the auction
      **/
      virtual std::string get_leader (void);

      /**
       * Sets the target of the distance calculations
       * @param target  the location that distance references are made to
       **/
      void set_target (pose::Position target);

      /**
      * Sets the target of the distance calculations
      * @param target  the GPSPosition that distance references are made to
      **/
      void set_target (utility::GPSPosition target);

      /**
      * Sets the platform whose frame we are referencing
      * @param platform  the hardware platform with the positioning frame
      **/
      void set_platform (platforms::BasePlatform * platform);

      /**
       * Calculate bids using current agent locations
       **/
      void calculate_bids (void);

    protected:

      /**
       * The location that distance will be calculated to
       **/
      pose::Position target_;

      /**
       * The platform is necessary to construct poses (we need frame)
       **/
      platforms::BasePlatform * platform_;
    };

    /**
    * Factory for creating minimum-bid auctions
    **/
    class GAMS_EXPORT AuctionMinimumDistanceFactory : public AuctionFactory
    {
    public:

      /**
      * Constructor
      **/
      AuctionMinimumDistanceFactory ();

      /**
      * Destructor
      **/
      virtual ~AuctionMinimumDistanceFactory ();

      /**
      * Creates a minimum-bid auction
      * @param auction_prefix the name of the auction (e.g. auction.position)
      * @param agent_prefix   the name of this bidder (e.g. agent.0)
      * @param knowledge      the knowledge base to use for syncing
      * @return  the new group
      **/
      virtual AuctionBase * create (const std::string & auction_prefix = "",
        const std::string & agent_prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0);
    };
  }
}

#endif // _GAMS_AUCTIONS_MINIMUM_DISTANCE_H_
