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
* @file ElectionFactoryRepository.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains the declaration of the repository for election factories
**/

#ifndef   _GAMS_ELECTIONS_ELECTION_FACTORY_REPOSITORY_H_
#define   _GAMS_ELECTIONS_ELECTION_FACTORY_REPOSITORY_H_

#include <vector>
#include <string>
#include <map>

#include "madara/knowledge/KnowledgeBase.h"
#include "ElectionFactory.h"

namespace gams
{
  namespace elections
  {
    /**
    * Convenience typedef for a map of types to factories
    **/
    typedef std::map <ElectionType, ElectionFactory *>  ElectionFactoryMap;

    /**
    * A repository for election factories.
    **/
    class GAMSExport ElectionFactoryRepository
    {
    public:
      /**
      * Constructor
      * @param  agent_prefix   the name of this voter (e.g. agent.0)
      * @param  knowledge      a knowledge base where election info is
      **/
      ElectionFactoryRepository (
        const std::string & agent_prefix,
        madara::knowledge::KnowledgeBase * knowledge = 0);

      /**
      * Destructor
      **/
      virtual ~ElectionFactoryRepository ();

      /**
      * Adds an algorithm factory
      * @param  type      the type for this factory
      * @param  factory   the factory for creating an algorithm
      * @return  the new algorithm
      **/
      void add (ElectionType type,
        ElectionFactory * factory);

      /**
      * Creates an election based on type
      * @param election_prefix the name of the election (e.g. election.position)
      * @return  the new election
      **/
      ElectionBase * create (const std::string & election_prefix);

      /**
      * Creates an election based on type
      * @param  type     the type of the election (@see ElectionTypes)
      * @return  the new election
      **/
      ElectionBase * create (ElectionType type);

      /**
      * Sets the knowledge base
      * @param  knowledge    the knowledge base to use
      **/
      void set_knowledge (madara::knowledge::KnowledgeBase * knowledge);

      /**
      * Sets the prefix for the current voting agent
      * @param prefix   the name of the agent (e.g. agent.0)
      **/
      void set_agent_prefix (const std::string & prefix);

    protected:

      /**
      * Initializes factories for all supported GAMS elections
      **/
      void init (void);

      /// knowledge base containing variables
      madara::knowledge::KnowledgeBase * knowledge_;

      /**
      * self prefix of the agent
      **/
      std::string agent_prefix_;

      /// a mapping of types to election factories
      ElectionFactoryMap factory_map_;

    };
  }
}

#include "ElectionFactoryRepository.inl"

#endif // _GAMS_ELECTIONS_ELECTION_FACTORY_REPOSITORY_H_
