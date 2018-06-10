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
 * @file FormationSync.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Algorithm for synchronized formation movements for groups and swarms
 **/

#ifndef _GAMS_ALGORITHMS_FORMATION_SYNC_H_
#define _GAMS_ALGORITHMS_FORMATION_SYNC_H_

#include <vector>
#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Barrier.h"
#include "gams/groups/GroupFactoryRepository.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for covering an area in formation with a synchronous
    * model of computation. Allows specification of arbitrary group or swarm.
    **/
    class GAMS_EXPORT FormationSync : public BaseAlgorithm
    {
    public:

      /**
       * Types of formations
       **/
      enum FormationTypes
      {
        PYRAMID,
        TRIANGLE,
        RECTANGLE,
        CIRCLE,
        LINE,
        WING
      };

      /**
       * Constructor
       * @param  start        the starting center of the formation
       * @param  end          the ending center of the formation
       * @param  group        the group name (e.g., group.allies)
       * @param  buffer       the distance between formation participants in
       *                      meters
       * @param  formation    type of formation (@see FormationTypes)
       * @param  barrier_name the barrier name to synchronize on
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       **/
      FormationSync (
        pose::Position & start,
        pose::Position & end,
        const std::string & group,
        double buffer,
        int formation,
        const std::string & barrier_name,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~FormationSync ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const FormationSync & rhs);
      
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
       * Generates the full plan
       * @param formation  the type of formation. @see FormationTypes
       **/
      void generate_plan (int formation);

      /**
       * Generates a position at an angle and distance
       * @param reference  the reference position
       * @param angle      the angle from the reference position
       * @param distance   the distance in meters
       * @return  the new position at the angle and distance
       **/
      pose::Position generate_position (pose::Position reference,
        double angle, double distance);

      /// center of formation start
      pose::Position start_;

      /// center of formation end
      pose::Position end_;

      /// factory for interacting with user-defined groups
      groups::GroupFactoryRepository group_factory_;

      /// the group that the user wishes the algorithm to use
      groups::GroupBase * group_;

      /// a convenience list of all current group members
      groups::AgentVector group_members_;

      /// the buffer between cells in the formation
      double buffer_;

      /// the planned positions of this agent
      std::vector <pose::Position> plan_;

      /// the formation to use
      int formation_;

      /// position in member assignment
      int position_;

      /// the move total before a pivot. Used for debugging
      int move_pivot_;

      /// movement barrier
      madara::knowledge::containers::Barrier barrier_;
    };
    
    /**
     * A factory class for creating Formation Coverage algorithms
     **/
    class GAMS_EXPORT FormationSyncFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Formation Coverage Algorithm.
       * @param   args      args come in pairs. The first arg is the
       *                    name of an arg. The second arg is the value
       *                    of the arg.<br>
       *                    start = starting coordinate location of formation<br>
       *                    end = ending coordinate location of formation<br>
       *                    group = name of the group in group.{name}.members<br>
       *                    buffer = buffer of the formation in meters<br>
       *                    formation = enum
       *                    @see FormationSync::FormationTypes<br>
       *                    barrier = unused variable to serve as barrier
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

#endif // _GAMS_ALGORITHMS_FORMATION_SYNC_H_
