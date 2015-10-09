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
 * @file CounterAlgorithm.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a helper class for counting the number of executions
 * of its various methods.
 **/

#ifndef   _GAMS_ALGORITHMS_COUNTER_H_
#define   _GAMS_ALGORITHMS_COUNTER_H_

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "madara/knowledge/containers/Integer.h"

namespace gams
{
  namespace algorithms
  {
    class CounterAlgorithm : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  knowledge    the context containing variables and values
       **/
      CounterAlgorithm (
        madara::knowledge::KnowledgeBase & knowledge);

      /**
       * Destructor
       **/
      ~CounterAlgorithm ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const CounterAlgorithm & rhs);
      
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
      
      /**
       * Resets all counters
       **/
      void reset_counters (void);
      
      /**
       * Enables all MADARA variable counters
       **/
      void enable_counters (void);
      
      /**
       * Disables all MADARA variable counters
       **/
      void disable_counters (void);

      /// enables the MADARA analyze counter
      bool enable_analyze_counter;
      
      /// enables the MADARA execute counter
      bool enable_execute_counter;
      
      /// enables the MADARA plan counter
      bool enable_plan_counter;

      /// tracks the number of loops executed with a normal 64 bit integer
      mutable madara::KnowledgeRecord::Integer  loops;

      /// tracks the number of calls to analyze
      mutable madara::knowledge::containers::Integer  analyze_counter;

      /// tracks the number of calls to execute
      mutable madara::knowledge::containers::Integer  execute_counter;

      /// tracks the number of calls to plan
      mutable madara::knowledge::containers::Integer  plan_counter;
    };
  }
}

#endif // _GAMS_ALGORITHMS_COUNTER_H_
