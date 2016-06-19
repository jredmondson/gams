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
* @file GroupTransient.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of a dynamic group that supports
* more transient memberships without a centralized omniscient group
* manager
**/

#ifndef   _GAMS_GROUPS_GROUP_TRANSIENT_H_
#define   _GAMS_GROUPS_GROUP_TRANSIENT_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/containers/Map.h"

#include "GroupBase.h"
#include "GroupFactory.h"

namespace gams
{
  namespace groups
  {
    /**
    * A fixed list of agent members
    **/
    class GAMSExport GroupTransient : public GroupBase
    {
    public:
      /**
      * Constructor. Note that if you do not specify prefix and
      * knowledge base, then you are essentially just working with
      * a local group with no connection to other agents.
      * @param prefix    the name of the group (e.g. group.protectors)
      * @param knowledge the knowledge base to use for syncing
      **/
      GroupTransient (const std::string & prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Constructor
      **/
      virtual ~GroupTransient ();

      /**
      * Adds the members to the group
      * @param  members  list of members to add to formation
      **/
      virtual void add_members (const AgentVector & members);

      /**
      * Clears the member list
      **/
      virtual void clear_members (void);

      /**
      * Retrieves the members from the group
      * @param  members  a list of the members currently in the group
      **/
      virtual void get_members (AgentVector & members) const;

      /**
      * Checks if the agent is a  member of the formation
      * @param  id     the agent id (e.g. agent.0 or agent.leader). If null,
      *                uses the current agent's id
      * @return  true if the agent is a member of the group
      **/
      virtual bool is_member (const std::string & id) const;

      /**
      * Writes the group information to a specified prefix
      * in a knowledge base. If no knowledge base is specified, then
      * saves in the original knowledge base. If no prefix is specified,
      * then saves in the original prefix location
      * @param prefix    the name of the group (e.g. group.protectors)
      * @param knowledge the knowledge base to save into
      **/
      virtual void write (const std::string & prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0) const;

      /**
      * Sets the prefix for the group in the knowledge base
      * @param prefix   the name of the group (e.g. group.protectors)
      * @param knowledge the knowledge base to use for syncing
      **/
      virtual void set_prefix (const std::string & prefix,
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Returns the number of members in the group
      * @return  the number of members
      **/
      virtual size_t size (void);

      /**
      * Syncs the list to the knowledge base
      **/
      virtual void sync (void);

    protected:

      /**
      * The knowledge base to use as a data plane
      **/
      madara::knowledge::KnowledgeBase * knowledge_;

      /**
      * The source member list in the knowledge base
      **/
      madara::knowledge::containers::Map members_;

      /**
      * member list for fast access
      **/
      AgentMap fast_members_;
    };

    /**
    * Factory for creating GroupTransient groups
    **/
    class GAMSExport GroupTransientFactory : public GroupFactory
    {
    public:

      /**
      * Constructor
      **/
      GroupTransientFactory ();

      /**
      * Destructor
      **/
      virtual ~GroupTransientFactory ();

      /**
      * Creates a group
      * @param  prefix       the name of the group (e.g. group.protectors)
      * @param  knowledge    the knowledge base of variables and values
      * @return  the new group
      **/
      virtual GroupBase * create (
        const std::string & prefix,
        madara::knowledge::KnowledgeBase * knowledge);
    };
  }
}

#endif // _GAMS_GROUPS_GROUP_TRANSIENT_H_
