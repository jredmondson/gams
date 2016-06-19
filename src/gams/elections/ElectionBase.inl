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

#ifndef   _GAMS_ELECTIONS_ELECTION_BASE_INL_
#define   _GAMS_ELECTIONS_ELECTION_BASE_INL_

#include "ElectionBase.h"
#include "gams/loggers/GlobalLogger.h"

inline int
gams::elections::ElectionBase::get_round (void) const
{
  return round_;
}

inline const std::string &
gams::elections::ElectionBase::get_agent_prefix (void) const
{
  return agent_prefix_;
}

inline void
gams::elections::ElectionBase::set_agent_prefix (const std::string & prefix)
{
  agent_prefix_ = prefix;
}

inline const std::string &
gams::elections::ElectionBase::get_election_prefix (void) const
{
  return election_prefix_;
}

inline void
gams::elections::ElectionBase::reset_votes_pointer (void)
{
  if (knowledge_ && election_prefix_ != "")
  {
    std::stringstream buffer;
    buffer << election_prefix_;
    buffer << ".";
    buffer << round_;
    votes_.set_name (buffer.str (), *knowledge_);
  }
}

inline void
gams::elections::ElectionBase::vote (const std::string & candidate,
  int votes)
{
  vote (agent_prefix_, candidate, votes);
}

#endif // _GAMS_ELECTIONS_ELECTION_BASE_INL_
