/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Swarm.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the swarm-prefixed MADARA variables
 **/

#ifndef   _GAMS_VARIABLES_SWARM_H_
#define   _GAMS_VARIABLES_SWARM_H_

#include <string>

#include "gams/GAMSExport.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/String.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/containers/Vector.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "AccentStatus.h"

namespace gams
{
  namespace variables
  {
    /**
    * A container for swarm information
    **/
    class GAMSExport Swarm
    {
    public:
      /**
       * Constructor
       **/
      Swarm ();

      /**
       * Destructor
       **/
      ~Swarm ();

      /**
       * Assignment operator
       * @param  device   device to copy
       **/
      void operator= (const Swarm & device);

      /**
       * Initializes variable containers
       * @param   knowledge  the variable context
       * @param   swarm_size size of the swarm
       **/
      void init_vars (madara::knowledge::KnowledgeBase & knowledge,
        const madara::knowledge::KnowledgeRecord::Integer& swarm_size = 1);
      
      /**
       * Initializes variable containers
       * @param   knowledge  the variable context
       * @param   swarm_size size of the swarm
       **/
      void init_vars (madara::knowledge::Variables & knowledge,
        const madara::knowledge::KnowledgeRecord::Integer& swarm_size = 1);

      /// the current command given to the swarm
      madara::knowledge::containers::String command;
      
      /// number of arguments for command
      madara::knowledge::containers::Vector command_args;
      
      /// minimum altitude for swarm to use
      madara::knowledge::containers::Double min_alt;

      /// the number of agents participating in the swarm
      madara::knowledge::containers::Integer size;
      
      /// container for accents
      AccentStatuses accents;

    protected:
      /// swarm command variable
      static const std::string SWARM_COMMAND;

      /// swarm min altitude variable
      static const std::string SWARM_MIN_ALT;

      /// swarm size variable
      static const std::string SWARM_SIZE;

      /**
       * Variable setup
       * @param  swarm_size   number of participating devices
       */
      void init_vars (const madara::knowledge::KnowledgeRecord::Integer& swarm_size);
    };
    
    /**
      * Initializes a swarm container
      * @param   variables  the variables to initialize
      * @param   knowledge  the knowledge base that houses the variables
      * @param   swarm_size size of the swarm
      **/
    GAMSExport void init_vars (Swarm & variables,
      madara::knowledge::KnowledgeBase & knowledge,
        const madara::knowledge::KnowledgeRecord::Integer& swarm_size = 1);
    
    /**
      * Initializes a swarm container
      * @param   variables  the variables to initialize
      * @param   knowledge  the knowledge base that houses the variables
      * @param   swarm_size size of the swarm
      **/
    GAMSExport void init_vars (Swarm & variables,
      madara::knowledge::Variables & knowledge,
        const madara::knowledge::KnowledgeRecord::Integer& swarm_size = 1);
  }
}

#endif // _GAMS_VARIABLES_SWARM_H_
