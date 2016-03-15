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
* @file AgentFormation.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the definition of the base agent formation class
**/

#ifndef   _GAMS_FORMATIONS_AGENT_FORMATION_H_
#define   _GAMS_FORMATIONS_AGENT_FORMATION_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/StringVector.h"
#include "gams/groups/GroupBase.h"
#include "gams/groups/GroupFactoryRepository.h"

namespace gams
{
  namespace formations
  {
    /**
    * An agent formation
    **/
    class AgentFormation
    {
    public:
      /**
       * Constructor
       **/
      AgentFormation ();

      /**
      * Constructor
      **/
      virtual ~AgentFormation ();

      /**
       * Configures the formation from arguments
       * @param  args   arguments to check through
       **/
      virtual void from_args (madara::knowledge::FunctionArguments & args) = 0;

      /**
      * Checks the goodness of an agent in the current formation.
      * @param  id      the agent id to check. Null means check all agents
      * @param  buffer  maximum allowed offset from correct location in meters
      * @return  the goodness of the formation, where 0 is bad and 1 is good
      **/
      virtual double goodness (
        const std::string & id = "", double buffer = 3.0) const = 0;

      /**
       * Sets the id of the current agent (e.g. "agent.0" or "agent.leader")
       * @param id    the current agent id
       **/
      void set_id (const std::string & id);

      /**
      * Checks if the agent is a member of the formation
      * @param  id     the agent id (e.g. agent.0 or agent.leader). If null,
      *                uses the current agent's id
      * @return  true if the agent is in the formation
      **/
      virtual bool is_member (const std::string & id = "") const = 0;

      /**
      * Checks if the agent is an extra member of the formation
      * @param  id     the agent id (e.g. agent.0 or agent.leader). If null,
      *                uses the current agent's id
      * @return  true if the agent is an extra member in the formation
      **/
      virtual bool is_extra (const std::string & id = "") const = 0;

    protected:

      /**
      * The knowledge base to use as a data plane
      **/
      madara::knowledge::KnowledgeBase * knowledge_;

      /**
       * The current agent id
       **/
      std::string id_;

      /**
       * The group factory
       **/
      groups::GroupFactoryRepository group_factory_;
    };
  }
}

#include "AgentFormation.inl"

#endif // _GAMS_FORMATIONS_AGENT_FORMATION_H_
