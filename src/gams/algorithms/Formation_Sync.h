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
 * @file Formation_Sync.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Algorithm for synchronized formation movements for groups and swarms
 **/

#ifndef _GAMS_ALGORITHMS_FORMATION_SYNC_H_
#define _GAMS_ALGORITHMS_FORMATION_SYNC_H_

#include <vector>
#include <string>

#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/variables/Algorithm_Status.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/utility/GPS_Position.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "madara/knowledge_engine/containers/Integer.h"
#include "madara/knowledge_engine/containers/Barrier.h"

namespace gams
{
  namespace algorithms
  {
    /**
    * An algorithm for covering an area in formation with a synchronous
    * model of computation. Allows specification of arbitrary group or swarm.
    **/
    class GAMS_Export Formation_Sync : public Base_Algorithm
    {
    public:

      /**
       * Types of formations
       **/
      enum Formation_Types
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
       * @param  members      the members of the formation
       * @param  buffer       the distance between formation participants in
       *                      meters
       * @param  formation    type of formation (@see Formation_Types)
       * @param  barrier_name the barrier name to synchronize on
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       **/
      Formation_Sync (
        utility::GPS_Position & start,
        utility::GPS_Position & end,
        const std::vector<std::string> & members,
        double buffer,
        int formation,
        std::string barrier_name,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
        platforms::Base_Platform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      ~Formation_Sync ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Formation_Sync & rhs);
      
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
       * @param formation  the type of formation. @see Formation_Types
       **/
      void generate_plan (int formation);

      /**
       * Generates a position at an angle and distance
       * @param reference  the reference position
       * @param angle      the angle from the reference position
       * @param distance   the distance in meters
       * @return  the new position at the angle and distance
       **/
      utility::GPS_Position generate_position (utility::GPS_Position reference,
        double angle, double distance);

      /**
       * Gets the position of the id in the member list
       * @param  id           the id of the member
       * @param  member_list  the list of members
       * @return the position of the id in the member list. -1 if not found.
       **/
      int get_position_in_member_list (std::string id,
        std::vector <std::string> & member_list);

      /// center of formation start
      utility::GPS_Position start_;

      /// center of formation end
      utility::GPS_Position end_;

      /// members of the formation (e.g., device.0, device.1, etc.)
      std::vector <std::string> members_;

      /// the buffer between cells in the formation
      double buffer_;

      /// the planned positions of this device
      std::vector <utility::GPS_Position> plan_;

      /// the formation to use
      int formation_;

      /// position in member assignment
      int position_;

      /// the move total before a pivot. Used for debugging
      int move_pivot_;

      /// movement barrier
      Madara::Knowledge_Engine::Containers::Barrier barrier_;
    };
    
    /**
     * A factory class for creating Formation Coverage algorithms
     **/
    class GAMS_Export Formation_Sync_Factory : public Algorithm_Factory
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
       *                    @see Formation_Sync::Formation_Types<br>
       *                    barrier = unused variable to serve as barrier
       * @param   knowledge the knowledge base to use
       * @param   platform  the platform. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       * @param   devices   the list of devices, which is dictated by
       *                    init_vars when a number of processes is set. This
       *                    will be set by the controller in init_vars
       **/
      virtual Base_Algorithm * create (
        const Madara::Knowledge_Vector & args,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        platforms::Base_Platform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Devices * devices);
    };
  }
}

#endif // _GAMS_ALGORITHMS_FORMATION_SYNC_H_
