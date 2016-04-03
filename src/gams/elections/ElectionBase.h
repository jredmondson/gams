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

#include "gams/GAMSExport.h"

namespace gams
{
  namespace elections
  {
    /// A vector of agent names
    typedef std::vector <std::string> AgentVector;

    /// A map of agent names
    typedef std::map <std::string,
      madara::knowledge::KnowledgeRecord::Integer> AgentMap;

    /**
    * Base class for an election
    **/
    class GAMSExport ElectionBase
    {
    public:
      /**
       * Constructor
       * @param name   the name of the election (e.g. election.protectors)
       * @param knowledge the knowledge base to use for syncing
       **/
      ElectionBase (const std::string & prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Constructor
      **/
      virtual ~ElectionBase ();

      /**
      * Adds the members to the election
      * @param  members  list of members to add
      **/
      virtual void add_members (const AgentVector & members) = 0;

      /**
      * Clears the member list
      **/
      virtual void clear_members (void) = 0;

      /**
      * Retrieves the members from the election
      * @param  members  a list of the members currently in the election
      **/
      virtual void get_members (AgentVector & members) const = 0;

      /**
      * Checks if the agent is a  member of the formation
      * @param  id     the agent id (e.g. agent.0 or agent.leader). If null,
      *                uses the current agent's id
      * @return  true if the agent is a member of the election
      **/
      virtual bool is_member (const std::string & id) const = 0;

      /**
      * Writes the election information to a specified prefix
      * in a knowledge base. If no knowledge base is specified, then
      * saves in the original knowledge base. If no prefix is specified,
      * then saves in the original prefix location
      * @param prefix    the name of the election (e.g. election.protectors)
      * @param knowledge the knowledge base to save into
      **/
      virtual void write (const std::string & prefix = "",
        madara::knowledge::KnowledgeBase * knowledge = 0) const = 0;

      /**
      * Sets the prefix for the election in the knowledge base
      * @param prefix   the name of the election (e.g. election.protectors)
      * @param knowledge the knowledge base to use for syncing
      **/
      virtual void set_prefix (const std::string & prefix,
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Returns the number of members in the election
      * @return  the number of members
      **/
      virtual size_t size (void) = 0;

      /**
      * Syncs the list to the knowledge base
      **/
      virtual void sync (void) = 0;

      /**
      * Gets the prefix for the election in the knowledge base
      * @return  the name of the election (e.g. election.protectors)
      **/
      const std::string & get_prefix (void);

    protected:

      /**
      * The knowledge base to use as a data plane
      **/
      mutable madara::knowledge::KnowledgeBase * knowledge_;

      /**
       * the prefix for the election
       **/
      std::string prefix_;
    };
  }
}

#include "ElectionBase.inl"

#endif // _GAMS_ELECTIONS_ELECTION_BASE_H_
