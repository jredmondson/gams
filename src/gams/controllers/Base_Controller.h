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
 * @file Base_Controller.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the preferred base controller for a GAMS control loop
 **/

#ifndef   _GAMS_BASE_CONTROLLER_H_
#define   _GAMS_BASE_CONTROLLER_H_

#include "gams/GAMS_Export.h"
#include "gams/variables/Device.h"
#include "gams/variables/Swarm.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Algorithm_Status.h"
#include "gams/variables/Platform_Status.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/algorithms/Controller_Algorithm_Factory.h"
#include "gams/platforms/Controller_Platform_Factory.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/platforms/Platform_Factory.h"

#ifdef _GAMS_JAVA_
#include <jni.h>
#endif

namespace gams
{
  namespace controllers
  {
    class GAMS_Export Base_Controller
    {
    public:
      /**
       * Constructor
       * @param   knowledge   The knowledge base to reference and mutate
       **/
      Base_Controller (Madara::Knowledge_Engine::Knowledge_Base & knowledge);

      /**
       * Destructor
       **/
      virtual ~Base_Controller ();

      /**
       * Defines the monitor function (the M of MAPE). This function should
       * return a 0 unless the MAPE loop should stop.
       **/
      virtual int monitor (void);

      /**
       * Analyzes the system to determine if platform or algorithm changes
       * are necessary. This function should
       * return a 0 unless the MAPE loop should stop.
       **/
      virtual int system_analyze (void);

      /**
       * Defines the analyze function (the A of MAPE). This function should
       * return a 0 unless the MAPE loop should stop.
       **/
      virtual int analyze (void);

      /**
       * Defines the plan function (the P of MAPE). This function should
       * return a 0 unless the MAPE loop should stop.
       **/
      virtual int plan (void);

      /**
       * Defines the execute function (the E of MAPE). This function should
       * return a 0 unless the MAPE loop should stop.
       **/
      virtual int execute (void);

      /**
       * Runs a single iteration of the MAPE loop
       * Always sends updates after the iteration.
       *
       * @return  the result of the MAPE loop iteration
       **/
      int run_once (void);

      /**
       * Runs iterations of the MAPE loop with specified periods
       * @param  loop_period  time (in seconds) between executions of the loop.
       *                      0 period is meant to run loop iterations as fast
       *                      as possible. Negative loop periods are invalid.
       * @param  max_runtime  maximum total runtime to execute the MAPE loops
       * @param  send_period  time (in seconds) between sending data.
       *                      If send_period <= 0, send period will use the
       *                      loop period.
       * @return  the result of the MAPE loop
       **/
      int run (double loop_period = 0.0,
        double max_runtime = -1,
        double send_period = -1.0);
      
      /**
       * Runs iterations of the MAPE loop with specified hertz
       * @param  loop_hz  the intended hz at which the loop should execute.
       *                  anything non-negative is valid. 0hz is treated as
       *                  as infinite hertz
       * @param  max_runtime  maximum total runtime to execute the MAPE loops
       * @param  send_hz  the intended hz at which updates should be sent. If
       *                  non-positive, loop_hz is used.
       * @return  the result of the MAPE loop
       **/
      inline int run_hz (double loop_hz = 0.0,
        double max_runtime = -1,
        double send_hz = -1.0)
      {
        double loop_rate, send_rate;

        // check for bad data
        if (loop_hz <= 0.0)
          loop_rate = 0.0;
        // otherwise, set rate to 1s divided by the intended hz
        else
          loop_rate = 1.0 / loop_hz;

        // copy from loop hz if send hz is non-positive
        if (send_hz <= 0)
          send_rate = loop_rate;
        // otherwise, set rate to 1s divided by the intended hz
        else
          send_hz = 1.0 / send_hz;

        return run (loop_rate, max_runtime, send_rate);
      }

      /**
       * Adds an accent algorithm
       * @param  algorithm   the name of the accent algorithm to add
       * @param  args        vector of knowledge record arguments
       **/
      void init_accent (const std::string & algorithm,
        const Madara::Knowledge_Vector & args = Madara::Knowledge_Vector ());

      /**
       * Clears all accent algorithms
       **/
      void clear_accents (void);

      /**
       * Adds an aliased platform factory. This factory will be
       * initialized with all appropriate variables in the
       * Platform_Factory class by the Base_Controller.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating a platform
       **/
      void add_platform_factory (
        const std::vector <std::string> & aliases,
        platforms::Platform_Factory * factory);
      
      /**
       * Adds an aliased algorithm factory. This factory will be
       * initialized with all appropriate variables in the
       * Algorithm_Factory class by the Base_Controller.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating an algorithm
       **/
      void add_algorithm_factory (
        const std::vector <std::string> & aliases,
        algorithms::Algorithm_Factory * factory);

      /**
       * Initializes an algorithm
       * @param  algorithm   the name of the algorithm to run
       * @param  args        vector of knowledge record arguments
       **/
      void init_algorithm (const std::string & algorithm,
        const Madara::Knowledge_Vector & args = Madara::Knowledge_Vector ());
 
      /**
       * Initializes the controller with a user-provided algorithm. This
       * algorithm's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  algorithm   the algorithm to use
       **/
      void init_algorithm (algorithms::Base_Algorithm * algorithm);

      /**
       * Initializes the platform
       * @param  platform   the name of the platform the controller is using
       * @param  args        vector of knowledge record arguments
       **/
      void init_platform (const std::string & platform,
        const Madara::Knowledge_Vector & args = Madara::Knowledge_Vector ());
       
      /**
       * Initializes the controller with a user-provided platform. This
       * platform's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  platform   the platform to use
       **/
      void init_platform (platforms::Base_Platform * platform);
           
#ifdef _GAMS_JAVA_
      /**
       * Initializes a Java-based algorithm
       * @param  algorithm  the java-based algorithm to use
       **/
      void init_algorithm (jobject algorithm);
      
      /**
       * Initializes a Java-based platform
       * @param  platform  the java-based platform to use
       **/
      void init_platform (jobject platform);
#endif

      /**
       * Initializes global variable containers
       * @param   id         node identifier
       * @param   processes  processes
       **/
      void init_vars (const Madara::Knowledge_Record::Integer & id = 0,
        const Madara::Knowledge_Record::Integer & processes = -1);
      
      /**
       * Initializes containers and knowledge base in a platform
       * This is usually the first thing a developer should do with
       * a user-defined platform.
       * @param   platform   the platform to initialize
       **/
      void init_vars (platforms::Base_Platform & platform);
      
      /**
       * Initializes containers and knowledge base in an algorithm.
       * This is usually the first thing a developer should do with
       * a user-defined algorithm.
       * @param   algorithm   the algorithm to initialize
       **/
      void init_vars (algorithms::Base_Algorithm & algorithm);

      /**
       * Gets the current algorithm
       * @return the algorithm
       **/
      algorithms::Base_Algorithm * get_algorithm (void);
      
      /**
       * Gets the current platform
       * @return the platform
       **/
      platforms::Base_Platform * get_platform (void);

    protected:

      /// Accents on the primary algorithm
      algorithms::Algorithms accents_;

      /// Algorithm to perform
      algorithms::Base_Algorithm * algorithm_;

      /// Containers for algorithm information
      variables::Algorithms algorithms_;
      
      /// Containers for device-related variables
      variables::Devices devices_;

      /// Knowledge base
      Madara::Knowledge_Engine::Knowledge_Base & knowledge_;

      /// Platform on which the controller is running
      platforms::Base_Platform * platform_;

      /// Containers for platform information
      variables::Platforms platforms_;

      /// Containers for self-referencing variables
      variables::Self self_;

      /// Containers for sensor information
      variables::Sensors sensors_;

      /// Containers for swarm-related variables
      variables::Swarm swarm_;

      /// Factory for creating new algorithms
      algorithms::Controller_Algorithm_Factory algorithm_factory_;

      /// Factory for creating new platforms
      platforms::Controller_Platform_Factory platform_factory_;

    private:

      /// Code shared between run and run_once
      int _run_once (void);
    };
  }
}

#endif // _GAMS_BASE_CONTROLLER_H_
