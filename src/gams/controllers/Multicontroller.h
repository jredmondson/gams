/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Multicontroller.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the multithreaded controller class declaration
 **/

#ifndef   _GAMS_CONTROLLERS_MULTICONTROLLER_H_
#define   _GAMS_CONTROLLERS_MULTICONTROLLER_H_

#include "gams/GamsExport.h"
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
#include "gams/controllers/BaseController.h"
#include "madara/transport/SharedMemoryPush.h"

#include "madara/knowledge/containers/String.h"
#include "madara/knowledge/containers/Vector.h"

#ifdef _GAMS_SCRIMMAGE_
#include <scrimmage/simcontrol/SimControl.h>
#endif

#ifdef _GAMS_JAVA_
#include <jni.h>
#endif

namespace gams
{
  namespace controllers
  {
    /**
     * A controller that has the capability of managing multiple controllers
     **/
    class GAMS_EXPORT Multicontroller
    {
    public:
      /**
       * Constructor
       * @param  num_controllers  the number of controllers to manage
       * @param  settings         settings to use
       **/
      Multicontroller(size_t num_controllers = 1,
        const ControllerSettings & settings = ControllerSettings());

      /**
       * Destructor
       **/
      virtual ~Multicontroller();

      /**
       * Adds an aliased algorithm factory. This factory will be
       * initialized with all appropriate variables in the
       * AlgorithmFactory class by the Multicontroller.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating an algorithm
       **/
      void add_algorithm_factory(
        const std::vector <std::string> & aliases,
        algorithms::AlgorithmFactory * factory);

      /**
       * Adds an aliased platform factory. This factory will be
       * initialized with all appropriate variables in the
       * PlatformFactory class by the Multicontroller.
       * @param  aliases   the named aliases for the factory. All
       *                   aliases will be converted to lower case
       * @param  factory   the factory for creating a platform
       **/
      void add_platform_factory(
        const std::vector <std::string> & aliases,
        platforms::PlatformFactory * factory);
      
      /**
       * Add transports to the knowledge bases that are being used
       * by each controller. If multicast or broadcast transports,
       * the hosts are simply copied. If using unicast-based connections,
       * then the first host in the settings is used and ports are
       * added in an ascending order for each subsequent KB. This
       * latter functionality is meant to aid in the expected use
       * case of the Multicontroller managing colocated agents that
       * need to communicate with each other. Note that you should
       * resize the Multicontroller to the appropriate number of
       * controllers BEFORE calling this function. Changes made
       * here are not retroactive for all future resizes. 
       * @param  source_settings  base transport settings to copy from
       **/
      void add_transports(
        const madara::transport::QoSTransportSettings & source_settings);

      /**
       * Evaluates a karl logic across all controllers and knowledge bases
       * @param logic  karl logic to evaluate
       * @param settings  evaluation settings for the logic evaluation
       **/
      void evaluate(const std::string & logic,
        const madara::knowledge::EvalSettings & settings =
          madara::knowledge::EvalSettings::DELAY);

      /**
       * Evaluates a karl logic across all controllers and knowledge bases
       * @param controller_index  the index of the managed controllers
       * @param logic             karl logic to evaluate
       * @param settings          evaluation settings for the logic evaluation
       **/
      void evaluate(size_t controller_index, const std::string & logic,
        const madara::knowledge::EvalSettings & settings =
          madara::knowledge::EvalSettings::DELAY);

      /**
       * Initializes all managed controllers with an accent algorithm
       * @param  algorithm   the name of the accent algorithm to add
       * @param  args        vector of knowledge record arguments
       **/
      void init_accent(
        const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap());

      /**
       * Adds an accent algorithm
       * @param  controller_index  the index of the managed controllers
       * @param  algorithm   the name of the accent algorithm to add
       * @param  args        vector of knowledge record arguments
       **/
      void init_accent(size_t controller_index,
        const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap());

      /**
       * Initializes all managed controllers with an algorithm
       * @param  algorithm   the name of the algorithm to run
       * @param  args        vector of knowledge record arguments
       **/
      void init_algorithm(
        const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap());
 
      /**
       * Initializes a specific controller with an algorithm
       * @param  controller_index  the index of the managed controllers
       * @param  algorithm   the name of the algorithm to run
       * @param  args        vector of knowledge record arguments
       **/
      void init_algorithm(size_t controller_index,
        const std::string & algorithm,
        const madara::knowledge::KnowledgeMap & args = madara::knowledge::KnowledgeMap());
 
      /**
       * Initializes the controller with a user-provided algorithm. This
       * algorithm's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  controller_index  the index of the managed controllers
       * @param  algorithm   the algorithm to use
       **/
      void init_algorithm(size_t controller_index,
        algorithms::BaseAlgorithm * algorithm);

      /**
       * Initializes all managed controllers with the platform
       * @param  platform   the name of the platform the controller is using
       * @param  args        vector of knowledge record arguments
       **/
      void init_platform(
        const std::string & platform,
        const madara::knowledge::KnowledgeMap & args =
          madara::knowledge::KnowledgeMap());
       
      /**
       * Initializes the platform
       * @param  controller_index  the index of the managed controllers
       * @param  platform   the name of the platform the controller is using
       * @param  args        vector of knowledge record arguments
       **/
      void init_platform(size_t controller_index,
        const std::string & platform,
        const madara::knowledge::KnowledgeMap & args =
          madara::knowledge::KnowledgeMap());
       
      /**
       * Initializes the controller with a user-provided platform. This
       * platform's memory will be maintained by the controller. DO NOT
       * DELETE THIS POINTER.
       * @param  controller_index  the index of the managed controllers
       * @param  platform   the platform to use
       **/
      void init_platform(size_t controller_index,
        platforms::BasePlatform * platform);
           
#ifdef _GAMS_JAVA_
      /**
       * Initializes a Java-based algorithm
       * @param controller_index  the index of the managed controllers
       * @param algorithm  the java-based algorithm to use
       **/
      void init_algorithm(size_t controller_index, jobject algorithm);
      
      /**
       * Initializes a Java-based platform
       * @param controller_index  the index of the managed controllers
       * @param platform  the java-based platform to use
       **/
      void init_platform(size_t controller_index, jobject platform);
#endif

      /**
       * Initializes global variable containers
       * @param   id         node identifier
       * @param   processes  processes
       **/
      void init_vars(const madara::knowledge::KnowledgeRecord::Integer & id = 0,
        const madara::knowledge::KnowledgeRecord::Integer & processes = -1);
      
      /**
       * Initializes containers and knowledge base in a platform
       * This is usually the first thing a developer should do with
       * a user-defined platform.
       * @param controller_index  the index of the managed controllers
       * @param   platform   the platform to initialize
       **/
      void init_vars(size_t controller_index,
        platforms::BasePlatform & platform);
      
      /**
       * Initializes containers and knowledge base in an algorithm.
       * This is usually the first thing a developer should do with
       * a user-defined algorithm.
       * @param controller_index  the index of the managed controllers
       * @param   algorithm   the algorithm to initialize
       **/
      void init_vars(size_t controller_index,
        algorithms::BaseAlgorithm & algorithm);

      /**
       * Gets the current algorithm at the controller index
       * @param controller_index  the index of the managed controllers
       * @return the algorithm
       **/
      algorithms::BaseAlgorithm * get_algorithm(size_t controller_index = 0);
      
      /**
       * Gets the current platform at the controller index
       * @param controller_index  the index of the managed controllers
       * @return the platform
       **/
      platforms::BasePlatform * get_platform(size_t controller_index = 0);

      /**
       * Returns the number of controllers being managed
       * @return the number of controllers managed by the multicontroller
       **/
      size_t get_num_controllers(void);

      /**
       * Returns the kb that is being used by the internal controller
       * @param controller_index  the index of the managed controllers
       * @return the knowledge base that is being referenced
       **/
      madara::knowledge::KnowledgeBase get_kb(size_t controller_index);

      /**
       * Clears all accent algorithms
       * @param  controller_index  the index of the managed controllers
       **/
      void clear_accents(size_t controller_index = 0);

      /**
       * Resizes the number of controllers and knowledge bases
       * @param   num_controllers  the number of controllers to manage
       **/
      void resize(size_t num_controllers);

      /**
       * Runs a single iteration of the MAPE loop
       * Always sends updates after the iteration.
       *
       * @return  the result of the MAPE loop iteration
       **/
      int run_once(void);

      /**
       * Runs iterations of the MAPE loop with configured settings
       * @return  the result of the MAPE loop
       **/
      int run(void);
      
      /**
       * Runs iterations of the MAPE loop with specified periods
       * @param  loop_period  time(in seconds) between executions of the loop.
       *                      0 period is meant to run loop iterations as fast
       *                      as possible. Negative loop periods are invalid.
       * @param  max_runtime  maximum total runtime to execute the MAPE loops
       * @param  send_period  time(in seconds) between sending data.
       *                      If send_period <= 0, send period will use the
       *                      loop period.
       * @return  the result of the MAPE loop
       **/
      int run(double loop_period,
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
      inline int run_hz(double loop_hz = 0.0,
        double max_runtime = -1,
        double send_hz = -1.0)
      {
        double loop_rate, send_rate;

        // check for bad data
        if(loop_hz <= 0.0)
          loop_rate = 0.0;
        // otherwise, set rate to 1s divided by the intended hz
        else
          loop_rate = 1.0 / loop_hz;

        // copy from loop hz if send hz is non-positive
        if(send_hz <= 0)
          send_rate = loop_rate;
        // otherwise, set rate to 1s divided by the intended hz
        else
          send_rate = 1.0 / send_hz;

        return run(loop_rate, max_runtime, send_rate);
      }

    protected:

      /// Controllers that need to be instrumented
      std::vector <BaseController *> controllers_;

      /// Knowledge base
      std::vector <madara::knowledge::KnowledgeBase> kbs_;

      /// transports meant for fast memory transport within the controller
      std::vector <madara::transport::SharedMemoryPush *> transports_;

      /// Settings for controller management and qos
      ControllerSettings settings_;
      
      // If using SCRIMMAGE
      scrimmage::SimControl sim_control_;
    };
  }
}

#endif // _GAMS_CONTROLLERS_MULTICONTROLLER_H_
