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
 * @file PlatformFactoryRepository.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Platform factory for the controller
 **/

#ifndef   _GAMS_PLATFORMS_PLATFORM_FACTORY_REPOSITORY_H_
#define   _GAMS_PLATFORMS_PLATFORM_FACTORY_REPOSITORY_H_

#include "gams/GAMSExport.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "PlatformFactory.h"

namespace gams
{
  namespace platforms
  {
    
    typedef std::map <std::string, PlatformFactory *>  FactoryMap;

    /**
     * The controller's platform factory
     **/
    class GAMSExport PlatformFactoryRepository
    {
    public:
      /**
       * Constructor
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       agent variables that describe self state
       **/
      PlatformFactoryRepository (
        madara::knowledge::KnowledgeBase * knowledge = 0,
        variables::Sensors * sensors = 0,
        variables::Platforms * platforms = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~PlatformFactoryRepository ();
      
      /**
       * Adds an algorithm factory
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating an algorithm
       * @return  the new algorithm
       **/
      void add (const std::vector <std::string> & aliases,
        PlatformFactory * factory);
      
      /**
       * Creates a platform
       * @param  type   type of platform to create
       * @param  args   a vector of Knowledge Record arguments
       * @return  the new platform
       **/
      BasePlatform * create (const std::string & type,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap ());
      
      /**
       * Sets the knowledge base
       * @param  knowledge    the knowledge base to use
       **/
      void set_knowledge (madara::knowledge::KnowledgeBase * knowledge);
      
      /**
       * Sets the map of platform names to platform information
       * @param  platforms   map of platform names to platform information
       **/
      void set_platforms (variables::Platforms * platforms);
      
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

    private:

      /// knowledge base containing variables
      madara::knowledge::KnowledgeBase * knowledge_;

      /// platform variables
      variables::Platforms * platforms_;

      /// self-referencing variables
      variables::Self * self_;

      /// sensor variables
      variables::Sensors * sensors_;

      /// a map of all aliases to factories
      FactoryMap  factory_map_;
    };

    /**
    * A globally accessible platform factory. The platform factory should
    * have all new platforms added to it before new threads that might access
    * the factory are started. This is not currently thread safe when adding
    * new platforms.
    **/
    extern GAMSExport madara::utility::Refcounter <PlatformFactoryRepository>
      global_platform_factory;
  }
}

#endif // _GAMS_PLATFORMS_PLATFORM_FACTORY_REPOSITORY_H_
