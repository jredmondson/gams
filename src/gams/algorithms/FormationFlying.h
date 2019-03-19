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
 * @file FormationFlying.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Declaration of FormationFlying class
 **/

#ifndef   _GAMS_ALGORITHMS_FORMATION_FLYING_H_
#define   _GAMS_ALGORITHMS_FORMATION_FLYING_H_

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/pose/Position.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/groups/GroupFactoryRepository.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for moving in formation
    **/
    class GAMS_EXPORT FormationFlying : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  head_id        target of the formation
       * @param  offset         offset of formation
       * @param  destination    destination of the formation
       * @param  group_name     group identifier(e.g. group.group1)
       * @param  modifier       modifier that influences the formation
       * @param  knowledge      the context containing variables and values
       * @param  platform       the underlying platform the algorithm will use
       * @param  sensors        map of sensor names to sensor information
       * @param  self           self-referencing variables
       **/
      FormationFlying(
        const std::string & head_id,
        const std::vector<double> & offset,
        const std::vector<double> & destination,
        const std::string & group_name,
        const std::string & modifier,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~FormationFlying();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator=(const FormationFlying & rhs);
      
      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze(void);
      
      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute(void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan(void);

      /**
       * Return true if this agent is head
       */
      bool is_head() const;

      /**
       * Get ready value
       * @return true if agents are in formation, false otherwise
       */
      bool is_ready() const;
      
    protected:
      /**
       * Get head's destination
       * @return utility::Position object of head's destination
       */
      pose::Position get_destination() const;

      /// formation wait string
      struct Compiled
      {
#ifndef _MADARA_NO_KARL_
        madara::knowledge::CompiledExpression ref;
#endif
        std::string agent;
      };
      std::vector<Compiled> compiled_formation_;

      /// are we in formation?
      madara::knowledge::containers::Integer formation_ready_;

      /// am i the head?
      bool head_;

      /// head id
      int head_id_;

      /// head location
      madara::knowledge::containers::NativeDoubleArray head_location_;

      /// head destination
      madara::knowledge::containers::NativeDoubleArray head_destination_;

      /// destination as Position
      pose::Position destination_;

      /// am i in formation?
      madara::knowledge::containers::Integer in_formation_;

      /// modifier enum
      enum
      {
        NONE,
        ROTATE
      } modifier_;

      /// do we need to move?
      bool need_to_move_;

      /// next position
      pose::Position next_position_;

      /// number of agents in formation; only head_id_ needs to know this
      unsigned int num_agents_;

      /// angular formation offsets
      double phi_;

      /// directional angular formation offsets
      double phi_dir_;

      /// planar distance formation offsets
      double rho_;

      /// list of sensor names
      variables::SensorNames sensor_names_;

      /// altitude formation offsets
      double z_;
    };
    
    /**
     * A factory class for creating Formation Flying algorithms
     **/
    class GAMS_EXPORT FormationFlyingFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Formation Flying Algorithm.
       * @param   args      args[0] = the target of the formation
       *                    args[1] = the cylindrical offset from the target
       *                    args[2] = the destination of the movement
       *                    args[3] = the number of members in the formation
       *                    args[4] = a modifier on the formation
       *                             (NONE or ROTATE)
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
      virtual BaseAlgorithm * create(
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        platforms::BasePlatform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Agents * agents);
    };
  }
}

#endif // _GAMS_ALGORITHMS_FORMATION_FLYING_H_
