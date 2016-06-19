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
 * @file KarlEvaluator.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Implementation of the Karl logic evaluator algorithm
 **/

#ifndef _GAMS_ALGORITHMS_KARL_H_
#define _GAMS_ALGORITHMS_KARL_H_

#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/algorithms/AlgorithmFactory.h"

#include "ace/High_Res_Timer.h"
#include "ace/OS_NS_sys_time.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm capable of executing other algorithms
    **/
    class GAMSExport KarlEvaluator : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  logic        a KaRL logic to evaluate
       * @param  store_result location in knowledge base to store result.
       *                      if empty, do not store the result
       * @param  is_wait      indicates if KaRL should wait for logic to
       *                      be true
       * @param  wait_time    maximum time to wait for logic to be true. -1
       *                      will wait forever.
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       * @param  agents       variables referencing agents
       **/
      KarlEvaluator (
        const std::string & logic,
        const std::string & store_result,
        bool is_wait,
        double wait_time,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Agents * agents = 0);

      /**
       * Destructor
       **/
      ~KarlEvaluator ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (KarlEvaluator & rhs);

      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute (void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan (void);

    protected:

      /// the compiled logic
      madara::knowledge::CompiledExpression  compiled_logic_;

      /// the evaluation settings
      madara::knowledge::EvalSettings settings_;

      /// original logic for debugging purposes
      std::string logic_;

      /// indicates if the logic should be evaluated as a wait statement
      bool is_wait_;

      /// indicates the time to wait. -1 means wait forever.
      double wait_time_;

      /// the end time
      ACE_Time_Value end_time_;
    };
    
    /**
     * A factory class for creating KarlEvaluator algorithms
     **/
    class GAMSExport KarlEvaluatorFactory : public AlgorithmFactory
    {
    public:
      /**
       * Creates an KarlEvaluator Algorithm.
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

#endif // _GAMS_ALGORITHMS_KARL_H_
