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
 * @file BaseController.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the preferred base controller for a GAMS control loop
 **/

#ifndef   _GAMS_BASE_CONTROLLER_H_
#define   _GAMS_BASE_CONTROLLER_H_

#include "ControllerSettings.h"

#include "gams/GAMSExport.h"
#include "gams/variables/Agent.h"
#include "gams/variables/Swarm.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/platforms/PlatformFactory.h"
#include "gams/groups/GroupBase.h"

#include "madara/knowledge/containers/String.h"
#include "madara/knowledge/containers/Vector.h"

#ifdef _GAMS_JAVA_
#include <jni.h>
#endif

namespace gams
{
  namespace controllers
  {
    /**
     * The basic controller that can be used to perform actions on platforms
     * and algorithms
     **/
    class GAMSExport BaseController
    {
    public:
      /**
       * Constructor
       * @param   knowledge   The knowledge base to reference and mutate
       * @param   settings    an initial configuration for the controller
       **/
      BaseController (madara::knowledge::KnowledgeBase & knowledge,
        const ControllerSettings & settings = ControllerSettings ());

      /**
       * Destructor
       **/
      virtual ~BaseController ();

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
      int run (double loop_period,
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
          send_rate = 1.0 / send_hz;

        return run (loop_rate, max_runtime, send_rate);
      }

      /**
       * Runs iterations of the MAPE loop with configured settings
       * @return  the result of the MAPE loop
       **/
      int run (void);

      /**
       * Adds an accent algorithm
       * @param  algorithm   the name of the accent algorithm to add
       * @param  args        vector of knowledge record arguments
       **/
      void init_accent (const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args =
          madara::knowledge::KnowledgeMap ());

      /**
       * Clears all accent algorithms
       **/
      void clear_accents (void);

      /**
       * Adds an aliased platform factory. This factory will be
       * initialized with all appropriate variables in the
       * PlatformFactory class by the BaseController.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating a platform
       **/
      void add_platform_factory (
        const std::vector <std::string> & aliases,
        platforms::PlatformFactory * factory);
      
      /**
       * Adds an aliased algorithm factory. This factory will be
       * initialized with all appropriate variables in the
       * AlgorithmFactory class by the BaseController.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating an algorithm
       **/
      void add_algorithm_factory (
        const std::vector <std::string> & aliases,
        algorithms::AlgorithmFactory * factory);

      /**
       * Initializes an algorithm
       * @param  algorithm   the name of the algorithm to run
       * @param  args        vector of knowledge record arguments
       **/
      void init_algorithm (const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap ());
 
      /**
       * Initializes the controller with a user-provided algorithm. This
       * algorithm's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  algorithm   the algorithm to use
       **/
      void init_algorithm (algorithms::BaseAlgorithm * algorithm);

      /**
       * Initializes the platform
       * @param  platform   the name of the platform the controller is using
       * @param  args        vector of knowledge record arguments
       **/
      void init_platform (const std::string & platform,
        const madara::knowledge::KnowledgeMap & args =
          madara::knowledge::KnowledgeMap ());
       
      /**
       * Initializes the controller with a user-provided platform. This
       * platform's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  platform   the platform to use
       **/
      void init_platform (platforms::BasePlatform * platform);
      
      /**
       * Configures the controller with initialization settings
       **/
      void configure (const ControllerSettings & settings);

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
      void init_vars (const madara::knowledge::KnowledgeRecord::Integer & id = 0,
        const madara::knowledge::KnowledgeRecord::Integer & processes = -1);

      /**
      * Initializes global variable containers
      * @param   self_prefix  the prefix of the agent in the knowledge base
      * @param   group_name   the name of the group that the agent listing
      *                       should come from
      **/
      void init_vars (const std::string & self_prefix,
        const std::string & group_name = "");

      /**
      * Initializes global variable containers
      * @param   self_prefix  the prefix of the agent in the knowledge base
      * @param   group        the group that should populate agents
      **/
      void init_vars (const std::string & self_prefix,
        const groups::GroupBase * group);

      /**
       * Initializes containers and knowledge base in a platform
       * This is usually the first thing a developer should do with
       * a user-defined platform.
       * @param   platform   the platform to initialize
       **/
      void init_vars (platforms::BasePlatform & platform);
      
      /**
       * Initializes containers and knowledge base in an algorithm.
       * This is usually the first thing a developer should do with
       * a user-defined algorithm.
       * @param   algorithm   the algorithm to initialize
       **/
      void init_vars (algorithms::BaseAlgorithm & algorithm);

      /**
       * Gets the current algorithm
       * @return the algorithm
       **/
      algorithms::BaseAlgorithm * get_algorithm (void);
      
      /**
       * Gets the current platform
       * @return the platform
       **/
      platforms::BasePlatform * get_platform (void);

    protected:

      /// Accents on the primary algorithm
      algorithms::Algorithms accents_;

      /// Algorithm to perform
      algorithms::BaseAlgorithm * algorithm_;

      /// Containers for algorithm information
      variables::Algorithms algorithms_;
      
      /// Containers for agent-related variables
      variables::Agents agents_;

      /// Knowledge base
      madara::knowledge::KnowledgeBase & knowledge_;

      /// Platform on which the controller is running
      platforms::BasePlatform * platform_;

      /// Containers for platform information
      variables::Platforms platforms_;

      /// Containers for self-referencing variables
      variables::Self self_;

      /// Containers for sensor information
      variables::Sensors sensors_;

      /// Containers for swarm-related variables
      variables::Swarm swarm_;

      /// Settings for controller management and qos
      ControllerSettings settings_;

      /// keeps track of the checkpoints saved in the control loop
      int checkpoint_count_;
    private:

      /// Code shared between run and run_once
      int run_once_ (void);
    };
  }
}

#endif // _GAMS_BASE_CONTROLLER_H_
