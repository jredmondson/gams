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

// create shortcut to refer to KnowledgeRecord
typedef   madara::knowledge::KnowledgeRecord  KnowledgeRecord;

gams::elections::ElectionBase::ElectionBase (const std::string & election_prefix,
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

void
gams::elections::ElectionBase::add_voters (groups::GroupBase * group)
{
  groups::AgentVector members;
  group->get_members (members);

  for (groups::AgentVector::iterator i = members.begin ();
    i != members.end (); ++i)
  {
    votes_.set (*i, "");
  }
}

bool
gams::elections::ElectionBase::has_voted (const std::string & agent_prefix)
{
  return votes_.has_prefix (agent_prefix);
}

gams::elections::CandidateList
gams::elections::ElectionBase::get_votes (
const std::string & id)
{
  return CandidateList ();
}

void
gams::elections::ElectionBase::set_election_prefix (const std::string & prefix)
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
