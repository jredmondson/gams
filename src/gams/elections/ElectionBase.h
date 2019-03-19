/**
* Copyright(c) 2016 Carnegie Mellon University. All Rights Reserved.
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
* @file ElectionBase.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of the base election class
**/

#ifndef   _GAMS_ELECTIONS_ELECTION_BASE_H_
#define   _GAMS_ELECTIONS_ELECTION_BASE_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/Map.h"

#include "ElectionTypesEnum.h"
#include "gams/groups/GroupBase.h"
#include "gams/GamsExport.h"

namespace gams
{
  namespace elections
  {
    /// list of candidates
    typedef  std::vector<std::string> CandidateList;

    /// candidate vote tally
    typedef std::map <std::string,
      madara::knowledge::KnowledgeRecord::Integer> CandidateVotes;

    /**
    * Base class for an election
    **/
    class GAMS_EXPORT ElectionBase
    {
    public:
      /**
      * Constructor
      * @param election_prefix the name of the election(e.g. election.leader)
      * @param agent_prefix   the name of this bidder(e.g. agent.0)
      * @param knowledge      the knowledge base to use for syncing
      **/
      ElectionBase(const std::string & election_prefix = "",
        const std::string & agent_prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Constructor
      **/
      virtual ~ElectionBase();

      /**
      * Checks if the agent has voted in this round
      * @param  agent_prefix  the participating agent's prefix(e.g. agent.0)
      * @return  true if the agent is a member of the election
      **/
      virtual bool has_voted(const std::string & agent_prefix);

      /**
      * Gets the votes cast in the election
      * @param  results  the results of the vote
      **/
      virtual void get_votes(CandidateVotes & results);

      /**
      * Gets the votes cast in the election by a specific group of agents
      * @param  group    the group that is of interest
      * @param  results  the results of the vote
      **/
      virtual void get_votes(
        groups::GroupBase * group, CandidateVotes & results);

      /**
      * Sets the prefix for the current bidding agent
      * @param prefix   the name of the agent(e.g. agent.0)
      **/
      virtual void set_agent_prefix(const std::string & prefix);

      /**
      * Sets the prefix for the election in the knowledge base
      * @param prefix   the name of the election(e.g. election.protectors)
      **/
      virtual void set_election_prefix(const std::string & prefix);

      /**
      * Sets the knowledge base
      * @param knowledge the knowledge base to use for syncing
      **/
      virtual void set_knowledge_base(
        madara::knowledge::KnowledgeBase * knowledge);

      /**
      * Syncs the election information from the knowledge base
      **/
      virtual void sync(void);

      /**
      * Gets the prefix for the current agent
      * @return  the name of this bidding agent(e.g. agent.0)
      **/
      const std::string & get_agent_prefix(void) const;

      /**
      * Gets the prefix for the election in the knowledge base
      * @return  the name of the election(e.g. election.protectors)
      **/
      const std::string & get_election_prefix(void) const;

      /**
      * Votes in the election. Uses the agent prefix that has been
      * set in this Election as the voter id.
      * @param  candidate   the candidate receiving votes
      * @param  votes       the number of votes cast
      **/
      void vote(const std::string & candidate, int votes = 1);

      /**
      * Bids in the election
      * @param  agent  the agent prefix who is bidding
      * @param  candidate   the candidate receiving votes
      * @param  votes       the number of votes cast
      **/
      virtual void vote(const std::string & agent,
        const std::string & candidate, int votes = 1);

      /**
      * Returns the leaders of the election in order of popularity
      * or whatever conditions constitute winning the election
      * @param  num_leaders maximum leaders to return
      * @return the leaders of the election up to num_leaders
      **/
      virtual CandidateList get_leaders(int num_leaders = 1) = 0;

      /**
      * Proceeds to the next election round in a multi-round
      * election
      **/
      virtual void advance_round(void);

      /**
      * Retrieves the round number, usually in a multi-round election
      * @return the agent prefix of the leader of the election
      **/
      int get_round(void) const;

      /**
      * Resets the round
      * @return the agent prefix of the leader of the election
      **/
      virtual void reset_round(void);

    protected:

      /**
      * calls a reset on the votes_ location in the knowledge base
      * using election_prefix_ + "." + round_.
      **/
      void reset_votes_pointer(void);

      /**
      * The knowledge base to use as a data plane
      **/
      mutable madara::knowledge::KnowledgeBase * knowledge_;

      /**
      * the prefix for the election
      **/
      std::string election_prefix_;

      /**
      * self prefix of the agent
      **/
      std::string agent_prefix_;

      /**
      * the election round in a multi-round election
      **/
      int round_;

      /**
      * convenience class for bids
      **/
      madara::knowledge::containers::Map votes_;
    };
  }
}

#include "ElectionBase.inl"

#endif // _GAMS_ELECTIONS_ELECTION_BASE_H_
