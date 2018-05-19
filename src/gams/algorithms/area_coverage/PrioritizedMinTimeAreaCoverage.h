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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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
 * @file PrioritizedMinTimeAreaCoverage.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * NOTE: Disabled pending repair of the variables::Sensor class
 **/

#ifndef _GAMS_ALGORITHMS_AREA_COVERAGE_PRIORITIZED_MIN_TIME_AREA_COVERAGE_H_
#define _GAMS_ALGORITHMS_AREA_COVERAGE_PRIORITIZED_MIN_TIME_AREA_COVERAGE_H_

#include "gams/algorithms/area_coverage/MinTimeAreaCoverage.h"

#include <string>
#include <set>

#include "madara/knowledge/KnowledgeUpdateSettings.h"
#include "gams/algorithms/AlgorithmFactory.h"

namespace gams
{
  namespace algorithms
  {
    namespace area_coverage
    {
      /**
      * Focuses on high priority areas over low priority, and attempts
      * to do minimum time coverage
      **/
      class GAMSExport PrioritizedMinTimeAreaCoverage : 
        public MinTimeAreaCoverage
      {
      public:
        /**
         * Constructor
         * @param  knowledge    the context containing variables and values
         * @param  search_id    search area identifier
         * @param  e_time       execution time
         * @param  platform     the underlying platform the algorithm will use
         * @param  sensors      map of sensor names to sensor information
         * @param  self         self-referencing variables
         * @param  agents      variables referencing agents
         * @param  algo_name    algorithm name
         **/
        PrioritizedMinTimeAreaCoverage (
          const std::string& search_id, 
          double e_time,
          madara::knowledge::KnowledgeBase * knowledge = 0,
          platforms::BasePlatform * platform = 0,
          variables::Sensors * sensors = 0,
          variables::Self * self = 0,
          variables::Agents * agents = 0,
          const std::string& algo_name = "pmtac");

        /**
         * Assignment operator
         * @param  rhs   values to copy
         **/
        void operator= (const PrioritizedMinTimeAreaCoverage & rhs);
  
      protected:
        /// get utility of moving from one index position to another
        virtual double get_utility (const utility::Position& start,
          const utility::Position& end, std::set<utility::Position>& online);
      }; // class PrioritizedMinTimeAreaCoverage

      /**
       * A factory class for creating prioritized minimum time
       * coverage algorithms
       **/
      class GAMSExport PrioritizedMinTimeAreaCoverageFactory
        : public AlgorithmFactory
      {
      public:

        /**
         * Creates a prioritized minimum time coverage algorithm
         * @param   args      args[0] = search area id
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
    } // namespace area_coverage
  } // namespace algorithms
} // namespace gams

#endif // _GAMS_ALGORITHMS_AREA_COVERAGE_PRIORITIZED_MIN_TIME_AREA_COVERAGE_H_
