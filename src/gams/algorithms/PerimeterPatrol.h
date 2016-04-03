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
 * @file PerimeterPatrol.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the perimeter patrol class. This
 * class is a rewrite of the PerimeterPatrolCoverage class with some
 * optimizations and options, and no baggage from BaseAreaCoverage
 **/

#ifndef   _GAMS_ALGORITHMS_PERIMETER_PATROL_H_
#define   _GAMS_ALGORITHMS_PERIMETER_PATROL_H_

#include "gams/algorithms/BaseAlgorithm.h"

#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "ace/High_Res_Timer.h"
#include "ace/OS_NS_sys_time.h"
#include "gams/utility/Position.h"
#include "gams/algorithms/AlgorithmFactory.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for patrolling a region
    **/
    class GAMSExport PerimeterPatrol : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  area         the region or search area to patrol
       * @param  max_time     the max time to run in secs (-1 for indefinite)
       * @param  counter      indicates if patrol should be counter clockwise
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       * @param  agents       variables referencing agents
       **/
      PerimeterPatrol (
        const std::string & area,
        double max_time,
        bool counter,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Agents * agents = 0);

      /**
       * Destructor
       **/
      ~PerimeterPatrol ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const PerimeterPatrol & rhs);
      
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

      /**
       * Generates locations from region information
       **/
      void generate_locations (void);

      /// the region/area to patrol
      std::string area_;

      /// the amount of time to spend patrolling in seconds
      double max_time_;

      /// patrol counter clockwise
      bool counter_;

      /// the end time
      ACE_Time_Value end_time_;

      /// the locations to visit
      utility::GpsPositions locations_;

      /// current location to move to
      size_t move_index_;

      /// indicates whether or not generate_locations has been succeeded
      bool initialized_;
    };

    /**
     * A factory class for creating PerimeterPatrol algorithms
     **/
    class GAMSExport PerimeterPatrolFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a PerimeterPatrol Algorithm.
       * @param   args      args[0] = type of movement
       *                    args[1] = number of move executions
       *                    args[2] = time to move in seconds
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

#endif // _GAMS_ALGORITHMS_PERIMETER_PATROL_H_
