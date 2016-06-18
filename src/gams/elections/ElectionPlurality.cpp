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

#include "ElectionPlurality.h"
#include "gams/loggers/GlobalLogger.h"
#include "madara/knowledge/containers/Integer.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

typedef  knowledge::KnowledgeRecord  KnowledgeRecord;

gams::elections::ElectionPluralityFactory::ElectionPluralityFactory ()
{
}

gams::elections::ElectionPluralityFactory::~ElectionPluralityFactory ()
{
}

gams::elections::ElectionBase *
gams::elections::ElectionPluralityFactory::create (
const std::string & election_prefix,
const std::string & agent_prefix,
madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::elections::ElectionPluralityFactory:" \
    " creating election from %s\n", election_prefix.c_str ());

  return new ElectionPlurality (election_prefix, agent_prefix, knowledge);
}

gams::elections::ElectionPlurality::ElectionPlurality (
  const std::string & election_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : ElectionBase (election_prefix, agent_prefix, knowledge)
{
}

/**
* Constructor
**/
gams::elections::ElectionPlurality::~ElectionPlurality ()
{

}

gams::elections::CandidateList
gams::elections::ElectionPlurality::get_leaders (int num_leaders)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::elections::ElectionPlurality:get_leaders" \
    " getting leaders from %s\n", election_prefix_.c_str ());

  CandidateList leaders;

  if (knowledge_)
  {
    knowledge::ContextGuard guard (*knowledge_);

    // create a list of votes for quick reference
    knowledge::VariableReferences votes;
    knowledge_->get_matches (votes_.get_name (), "", votes);

    typedef std::map <KnowledgeRecord::Integer, CandidateList> Leaderboard;

    CandidateVotes candidates;
    Leaderboard leaderboard;
    std::string last_voter;

    // tally the votes
    for (knowledge::VariableReferences::const_iterator i = votes.begin ();
      i != votes.end (); ++i)
    {
      std::string ballot_string = i->get_name ();
      std::string::size_type delimiter_pos = ballot_string.find ("->");

      if (delimiter_pos != std::string::npos)
      {
        std::string candidate = ballot_string.substr (delimiter_pos + 2);
        std::string voter = ballot_string.substr (
          votes_.get_name ().size () + 1, delimiter_pos);

        // plurality votes only allow one vote per voter
        if (voter != last_voter)
        {
          candidates[candidate] += 1;
          last_voter = voter;
        }
      }
    }

    // construct the leaderboard
    for (CandidateVotes::iterator i = candidates.begin ();
      i != candidates.end (); ++i)
    {
      leaderboard[i->second].push_back (i->first);
    }

    // leaderboard is in ascending order, so grab from the back
    for (Leaderboard::reverse_iterator i = leaderboard.rbegin ();
      i != leaderboard.rend () && (int) leaders.size () < num_leaders; ++i)
    {
      // if it is a tie, we could provide more than num_leaders
      leaders.insert (leaders.end (), i->second.begin (), i->second.end ());
    }
  }

  return leaders;
}

std::string
gams::elections::ElectionPlurality::get_leader (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::elections::ElectionPlurality:get_leader" \
    " getting leader from %s\n", election_prefix_.c_str ());

  std::string leader;
  CandidateList leaders = get_leaders ();

  if (leaders.size () > 0)
  {
    leader = leaders[0];
  }

  return leader;
}
