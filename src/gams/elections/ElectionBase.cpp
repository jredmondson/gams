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

#include "ElectionBase.h"
#include <sstream>

// create shortcuts
namespace  knowledge = madara::knowledge;
typedef    knowledge::KnowledgeRecord  KnowledgeRecord;

gams::elections::ElectionBase::ElectionBase (
  const std::string & election_prefix,
  const std::string & agent_prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : knowledge_ (knowledge),
  election_prefix_ (election_prefix),
  agent_prefix_ (agent_prefix),
  round_ (0)
{
  reset_votes_pointer ();
}

gams::elections::ElectionBase::~ElectionBase ()
{
}

bool
gams::elections::ElectionBase::has_voted (const std::string & agent_prefix)
{
  bool result (false);

  if (knowledge_)
  {
    knowledge::ContextGuard guard (*knowledge_);

    // create a list of votes for quick reference
    knowledge::VariableReferences votes;
    knowledge_->get_matches (
      votes_.get_name () + "." + agent_prefix + "->", "", votes);

    result = votes.size () != 0;
  }

  return result;
}

void
gams::elections::ElectionBase::get_votes (CandidateVotes & results)
{
  results.clear ();

  if (knowledge_ && election_prefix_ != "")
  {
    // create a list of votes for quick reference
    knowledge::VariableReferences votes;
    knowledge_->get_matches (votes_.get_name (), "", votes);

    // tally the votes
    for (knowledge::VariableReferences::const_iterator i = votes.begin ();
      i != votes.end (); ++i)
    {
      std::string ballot_string = i->get_name ();
      std::string::size_type delimiter_pos = ballot_string.find ("->");

      if (delimiter_pos != std::string::npos)
      {
        std::string candidate = ballot_string.substr (delimiter_pos + 2);

        // add the votes to the results
        results[candidate] += knowledge_->get (*i).to_integer ();
      }
    }
  }
}

void
gams::elections::ElectionBase::get_votes (
  groups::GroupBase * group, CandidateVotes & results)
{
  results.clear ();

  if (knowledge_ && election_prefix_ != "")
  {
    // create a list of votes for quick reference
    knowledge::VariableReferences votes;
    knowledge_->get_matches (votes_.get_name (), "", votes);

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

        if (group->is_member (voter))
        {
          // add the votes to the results
          results[candidate] += knowledge_->get (*i).to_integer ();
        }
      }
    }
  }
}

void
gams::elections::ElectionBase::set_election_prefix (
  const std::string & prefix)
{
  election_prefix_ = prefix;
  reset_votes_pointer ();
}

void
gams::elections::ElectionBase::set_knowledge_base (
madara::knowledge::KnowledgeBase * knowledge)
{
  knowledge_ = knowledge;
  reset_votes_pointer ();
}

void
gams::elections::ElectionBase::sync (void)
{
  votes_.sync_keys ();
}

void
gams::elections::ElectionBase::vote (const std::string & agent,
const std::string & candidate, int votes)
{
  std::stringstream buffer;
  buffer << agent;
  buffer << "->";
  buffer << candidate;
  votes_.set (buffer.str (), KnowledgeRecord::Integer (votes));
}

void gams::elections::ElectionBase::advance_round (void)
{
  ++round_;
  reset_votes_pointer ();
}

void gams::elections::ElectionBase::reset_round (void)
{
  round_ = 0;
  reset_votes_pointer ();
}
