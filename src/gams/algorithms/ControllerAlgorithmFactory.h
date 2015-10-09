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
 * @file ControllerAlgorithmFactory.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the algorithm factory
 **/

#ifndef   _GAMS_ALGORITHMS_CONTROLLER_ALGORITHM_FACTORY_H_
#define   _GAMS_ALGORITHMS_CONTROLLER_ALGORITHM_FACTORY_H_

#include "gams/GAMSExport.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Device.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "AlgorithmFactory.h"

namespace gams
{
  namespace algorithms
  {
    typedef std::map <std::string, AlgorithmFactory *>  FactoryMap;

    /**
     * The controller's algorithm factory
     **/
    class GAMSExport ControllerAlgorithmFactory
    {
    public:
      /**
       * Constructor
       * @param  knowledge  the knowledge base
       * @param  sensors    variables for all sensors
       * @param  platform   the current platform
       * @param  self       self-referencing variables
       * @param  devices    devices of the swarm
       **/
      ControllerAlgorithmFactory (
        madara::knowledge::KnowledgeBase * knowledge = 0,
        variables::Sensors * sensors = 0,
        platforms::BasePlatform * platform = 0,
        variables::Self * self = 0,
        variables::Devices * devices = 0);

      /**
       * Destructor
       **/
      ~ControllerAlgorithmFactory ();
      
      /**
       * Adds an algorithm factory
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating an algorithm
       * @return  the new algorithm
       **/
      void add (const std::vector <std::string> & aliases,
        AlgorithmFactory * factory);
      
      /**
       * Creates an algorithm
       * @param  type   type of algorithm to create
       * @param  args   a vector of Knowledge Record arguments
       * @return  the new algorithm
       **/
      BaseAlgorithm * create (const std::string & type,
        const madara::KnowledgeVector & args = madara::KnowledgeVector ());
      
      /**
       * Sets list of devices participating in swarm
       * @param  devices    devices in the swarm
       **/
      void set_devices (variables::Devices * devices);
      
      /**
       * Sets the knowledge base
       * @param  knowledge    the knowledge base to use
       **/
      void set_knowledge (madara::knowledge::KnowledgeBase * knowledge);
      
      /**
       * Sets the map of platform names to platform information
       * @param  platform   the platform to use
       **/
      void set_platform (platforms::BasePlatform * platform);
      
      /**
       * Sets self-referencing variables
       * @param  self       self-referencing variables
       **/
      void set_self (variables::Self * self);
      
      /**
       * Sets the map of sensor names to sensor information
       * @param  sensors      map of sensor names to sensor information
       **/
      void set_sensors (variables::Sensors * sensors);
      
      /**
       * Initializes factories for all supported GAMS algorithms
       **/
      void initialize_default_mappings (void);

    protected:

      /// list of devices participating in the swarm
      variables::Devices * devices_;

      /// knowledge base containing variables
      madara::knowledge::KnowledgeBase * knowledge_;

      /// platform variables
      platforms::BasePlatform * platform_;

      /// self-referencing variables
      variables::Self * self_;

      /// sensor variables
      variables::Sensors * sensors_;

      /// a map of all aliases to factories
      FactoryMap  factory_map_;
    };
  }
}

#endif // _GAMS_ALGORITHMS_CONTROLLER_ALGORITHM_FACTORY_H_
