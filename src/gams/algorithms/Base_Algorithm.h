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
 * @file Base.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the base algorithm class
 **/

#ifndef   _GAMS_ALGORITHMS_BASE_H_
#define   _GAMS_ALGORITHMS_BASE_H_

#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/variables/Algorithm.h"
#include "gams/variables/Self.h"
#include "gams/utility/Region.h"
#include "madara/knowledge_engine/Knowledge_Base.h"

#include <vector>

namespace gams
{
  namespace controllers
  {
    class Base;
  }

  namespace algorithms
  {
    /**
     * Possible algorithm statuses, as returnable by analyze ()
     **/
    enum Status
    {
      UNKNOWN = 0,
      OK  = 1,
      WAITING = 2,
      DEADLOCKED = 4,
      FAILED = 8
    };

    class GAMS_Export Base
    {
    public:
      // allow Base controller to initialize our variables
      friend class controllers::Base;

      /**
       * Constructor
       * @param  knowledge    the knowledge base of variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables for this device
       * @param  devices      list of devices in the swarm
       **/
      Base (
        Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
        platforms::Base * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Devices * devices = 0);

      /**
       * Destructor
       **/
      virtual ~Base ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Base & rhs);
      
      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze () = 0;
      
      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute () = 0;

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan () = 0;
      
      /**
       * Sets the list of devices in the swarm
       * @param  devices      list of devices
       **/
      virtual void set_devices (variables::Devices * devices);
      
      /**
       * Sets the platform
       * @param  platform     the underlying platform the algorithm will use
       **/
      virtual void set_platform (platforms::Base * platform);

      /**
       * Sets the map of sensor names to sensor information
       * @param  self      pointer to self-referencing variables container
       **/
      virtual void set_self (variables::Self * self);
      
      /**
       * Sets the map of sensor names to sensor information
       * @param  sensors      map of sensor names to sensor information
       **/
      virtual void set_sensors (variables::Sensors * sensors);
      
      /**
       * Gets the list of devices
       **/
      variables::Devices * get_devices (void);

      /**
       * Gets the knowledge base
       **/
      Madara::Knowledge_Engine::Knowledge_Base * get_knowledge_base (void);

      /**
       * Gets the platform
       **/
      platforms::Base * get_platform (void);

      /**
       * Gets self-defined variables
       **/
      variables::Self * get_self (void);

      /**
       * Gets the available sensor information
       **/
      variables::Sensors * get_sensors (void);

      /**
       * Gets algorithm status variables
       **/
      variables::Algorithm * get_algorithm_status (void);

    protected:
      /// the list of devices potentially participating in the algorithm
      variables::Devices * devices_;

      /// number of executions
      unsigned int executions_;

      /// provides access to the knowledge base
      Madara::Knowledge_Engine::Knowledge_Base * knowledge_;

      /// provides access to the platform
      platforms::Base * platform_;

      /// the algorithm's concept of self
      variables::Self * self_;

      /// provides access to sensor information
      variables::Sensors * sensors_;

      /// provides access to status information for this platform
      variables::Algorithm status_;
    };

    typedef  std::vector <Base *>   Algorithms;
  }
}

#endif // _GAMS_VARIABLES_SWARM_H_
