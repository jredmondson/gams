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
 * @file GroupBarrier.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Algorithm for synchronized formation movements for groups and swarms
 **/

#ifndef _GAMS_ALGORITHMS_GROUP_BARRIER_H_
#define _GAMS_ALGORITHMS_GROUP_BARRIER_H_

#include <vector>
#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/utility/GPSPosition.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Barrier.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for covering an area in formation with a synchronous
    * model of computation. Allows specification of arbitrary group or swarm.
    **/
    class GAMSExport GroupBarrier : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  members      the members of the formation
       * @param  barrier_name the barrier name to synchronize on
       * @param  interval     interval in seconds between barrier increments
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       **/
      GroupBarrier (
        const std::vector<std::string> & members,
        std::string barrier_name,
        double interval,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~GroupBarrier ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const GroupBarrier & rhs);
      
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
       * Gets the position of the id in the member list
       * @param  id           the id of the member
       * @param  member_list  the list of members
       * @return the position of the id in the member list. -1 if not found.
       **/
      int get_position_in_member_list (std::string id,
        std::vector <std::string> & member_list);

      /// members of the formation (e.g., agent.0, agent.1, etc.)
      std::vector <std::string> members_;

      /// position in member assignment
      int position_;

      /// movement barrier
      madara::knowledge::containers::Barrier barrier_;

      // next barrier time
      ACE_Time_Value next_barrier_;

      // the epoch between barriers
      ACE_Time_Value interval_;

    };
    
    /**
     * A factory class for creating Formation Coverage algorithms
     **/
    class GAMSExport GroupBarrierFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Formation Coverage Algorithm.
       * @param   args      args come in pairs. The first arg is the
       *                    name of an arg. The second arg is the value
       *                    of the arg.<br>
       *                    group = name of the group in group.{name}.members<br>
       *                    barrier = unused variable to serve as barrier
       *                    interval = interval in seconds to wait before
       *                               moving between barrier rounds
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

#endif // _GAMS_ALGORITHMS_GROUP_BARRIER_H_
