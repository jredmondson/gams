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
 * @file Follow.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the declaration of the Follow algorithm
 **/

#ifndef   _GAMS_ALGORITHMS_FOLLOW_H_
#define   _GAMS_ALGORITHMS_FOLLOW_H_

#include <queue>

#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/utility/GPSPosition.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/utility/CartesianFrame.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for following a target
    **/
    class GAMSExport Follow : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  target     full agent name the agent should follow
       * @param  offset     offset from target.location to move to
       * @param  knowledge  the context containing variables and values
       * @param  platform   the underlying platform the algorithm will use
       * @param  sensors    map of sensor names to sensor information
       * @param  self       self-referencing variables
       **/
      Follow (
        const std::string & target,
        const std::vector <double> & offset,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);
      
      /**
       * Destructor
       **/
      ~Follow ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Follow & rhs);
      
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
      /// location of agent to follow
      variables::Agent target_;

      /// current target location shared between analyze and execute
      utility::Location target_location_;

      /// current target location shared between analyze and execute
      utility::Location target_destination_;

      /// current target location shared between analyze and execute
      utility::Location last_target_destination_;

      /// keep track of last location
      utility::Location last_location_;

      /// keep track of last leader/target location
      utility::Location target_last_location_;

      /// keep track of the target orientation
      utility::Orientation target_orientation_;

      /// keep track of the offset that agent should be at
      std::vector <double> offset_;

      /// keep track of the minimum_buffer that should be maintained
      std::vector <double> minimum_buffer_;

      /// flag between analyze and execute indicating new move is necessary
      bool need_move_;

      /// keep track of whether we had previously calculated target orient
      bool had_valid_dest_orientation_;

      /// last valid destination based orientation
      gams::utility::Orientation last_dest_orientation;
    };

    /**
     * A factory class for creating Follow Algorithms
     **/
    class GAMSExport FollowFactory : public AlgorithmFactory
    {
    public:

      /**
       * Creates a Follow Algorithm.
       * @param   args      target = the target to follow
       *                    delay = the time step delay
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

#endif // _GAMS_ALGORITHMS_FOLLOW_H_
