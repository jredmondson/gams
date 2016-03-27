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
#include <time.h>

#include "madara/utility/Utility.h"
#include "madara/knowledge/containers/Integer.h"

#include "GroupTransient.h"
#include "gams/loggers/GlobalLogger.h"


namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

gams::groups::GroupTransientFactory::GroupTransientFactory ()
{
}

gams::groups::GroupTransientFactory::~GroupTransientFactory ()
{
}

gams::groups::GroupBase *
gams::groups::GroupTransientFactory::create (
const std::string & prefix,
madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransientFactory:" \
    " creating group from %s\n", prefix.c_str ());

  return new GroupTransient (prefix, knowledge);
}

gams::groups::GroupTransient::GroupTransient (const std::string & prefix,
  madara::knowledge::KnowledgeBase * knowledge)
  : GroupBase (prefix, knowledge)
{
  if (knowledge && prefix != "")
  {
    members_.set_name (prefix + ".members", *knowledge);
    sync ();
  }
}

gams::groups::GroupTransient::~GroupTransient ()
{
}

void
gams::groups::GroupTransient::add_members (const AgentVector & members)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:add_members" \
    " adding %d members\n", (int)members.size ());

  knowledge::KnowledgeRecord::Integer cur_time =
    (knowledge::KnowledgeRecord::Integer)time (NULL);

  bool update_knowledge = knowledge_ && prefix_ != "";

  // add the members to the underlying knowledge base
  for (size_t i = 0; i < members.size (); ++i)
  {
    const std::string id = members[i];

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::groups::GroupTransient:add_members" \
      " adding member %s to fast map\n", id.c_str ());

    fast_members_[id] = cur_time;

    if (update_knowledge)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::groups::GroupTransient:add_members" \
        " adding member %s to %s\n", id.c_str (), prefix_.c_str ());

      members_.set (id, cur_time);
    }
  }
}

void
gams::groups::GroupTransient::clear_members (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:clear_members" \
    " clearing all %d members\n", (int)fast_members_.size ());

  fast_members_.clear ();
  members_.clear ();
}

void
gams::groups::GroupTransient::get_members (AgentVector & members) const
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:get_members" \
    " retrieving member list\n");

  members.resize (fast_members_.size ());

  // iterate through the agent map and copy over the member info
  size_t j = 0;
  for (AgentMap::const_iterator i = fast_members_.begin ();
    i != fast_members_.end (); ++i, ++j)
  {
    members[j] = i->first;
  }
}

bool
gams::groups::GroupTransient::is_member (const std::string & id) const
{
  return fast_members_.find (id) != fast_members_.end ();
}

size_t
gams::groups::GroupTransient::size (void)
{
  return fast_members_.size ();
}

void
gams::groups::GroupTransient::set_prefix (const std::string & prefix,
  madara::knowledge::KnowledgeBase * knowledge)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:set_prefix" \
    " setting prefix to %s\n", prefix.c_str ());

  GroupBase::set_prefix (prefix, knowledge);

  if (knowledge && prefix != "")
  {
    members_.set_name (prefix + ".members", *knowledge);
    sync ();
  }
}

void
gams::groups::GroupTransient::sync (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:sync" \
    " syncing with %s source\n", prefix_.c_str ());

  if (knowledge_ && prefix_ != "")
  {
    // sync with map is always expensive. Sync and clear
    members_.sync_keys ();
    fast_members_.clear ();

    // get the new list of keys
    std::vector <std::string> keys;
    members_.keys (keys);
    
    // add the keys with values to the AgentMap fast_members_
    for (size_t i = 0; i < keys.size (); ++i)
    {
      const std::string key = keys[i];
      fast_members_[key] = members_[key].to_integer ();
    }
  }
}

void
gams::groups::GroupTransient::write (const std::string & prefix,
madara::knowledge::KnowledgeBase * knowledge) const
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::groups::GroupTransient:write" \
    " performing initialization checks\n");

  std::string location;
  bool is_done (false);

  // set location in knowledge base to write to
  if (prefix == "")
  {
    if (prefix_ != "")
    {
      location = prefix_;
    }
    else
    {
      is_done = true;
    }
  }
  else
  {
    location = prefix;
  }

  // set knowledge base to write to
  if (!is_done && !knowledge)
  {
    if (knowledge_)
    {
      knowledge = knowledge_;
    }
    else
    {
      is_done = true;
    }
  }

  // if we have a valid location and knowledge base, continue
  if (!is_done)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::groups::GroupTransient:write" \
      " writing group information to %s\n", location.c_str ());

    // create members and type. Note we set type to 1.
    containers::Map members (location + ".members", *knowledge);
    containers::Integer type (location + ".type", *knowledge, 1);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::groups::GroupTransient:write" \
      " writing %d members to %s.members\n",
      (int)fast_members_.size (), location.c_str ());

    // iterate through the fast members list and set members at location
    for (AgentMap::const_iterator i = fast_members_.begin ();
      i != fast_members_.end (); ++i)
    {
      members.set (i->first, i->second);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::groups::GroupTransient:write" \
      " no valid prefix to write to. No work to do.\n");
  }
}
