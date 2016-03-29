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
 * @file Executor.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Implementation of the Executor algorithm
 **/

#ifndef _GAMS_ALGORITHMS_EXECUTOR_H_
#define _GAMS_ALGORITHMS_EXECUTOR_H_

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/algorithms/FormationFlying.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/algorithms/AlgorithmFactoryRepository.h"

namespace gams
{
  namespace algorithms
  {
    /**
     * A helper class for algorithm information
     **/
    class GAMSExport AlgorithmMetaData
    {
    public:
      /// a KaRL precondition
      std::string precond;

      /// the id of the algorithm to create
      std::string id;

      /// arguments to the algorithm
      madara::knowledge::KnowledgeMap args;

      /// maximum time to run the algorithm
      double max_time;
    };

    /// helper typdef for collection of AlgorithMetaData
    typedef  std::vector <AlgorithmMetaData> AlgorithmMetaDatas;

    /**
    * An algorithm capable of executing other algorithms
    **/
    class GAMSExport Executor : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  algorithms   list of algorithms to execute
       * @param  repeat       number of times to repeat. -1 is infinite
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       * @param  agents       variables referencing agents
       **/
      Executor (
        AlgorithmMetaDatas algorithms,
        int repeat,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Agents * agents = 0);

      /**
       * Destructor
       **/
      ~Executor ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (Executor & rhs);

      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void);

      /**
       * Executes the next step
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute (void);

      /**
       * Plans the next execution
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan (void);

    protected:

      /// for keeping track of algorithms
      AlgorithmMetaDatas algorithms_;

      /// number of times to repeat
      int repeat_;

      /// current algorithm
      size_t alg_index_;

      /// tracks the number of cycles completed through algorithms
      int cycles_;

      /// current executing algorithm
      BaseAlgorithm * current_;

      /// the end time
      ACE_Time_Value end_time_;

      /// indicates if the precondition has been met for current algorithm
      bool precond_met_;
    };
    
    /**
     * A factory class for creating Executor algorithms
     **/
    class GAMSExport ExecutorFactory : public AlgorithmFactory
    {
    public:
      /**
       * Creates an Executor Algorithm.
       * @param   args      arguments to the executor algorithm
       * @param   knowledge the knowledge base to use
       * @param   platform  the platform. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       * @param   agents   the list of agents, which is dictated by
       *                    init_vars when a number of processes is set. This
       *                    will be set by the controller in init_vars
       **/
      virtual BaseAlgorithm * create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        platforms::BasePlatform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Agents * agents);
    };
  }
}

#endif // _GAMS_ALGORITHMS_EXECUTOR_H_
