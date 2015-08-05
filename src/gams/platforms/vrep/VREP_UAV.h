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
 * @file VREP_UAV.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the VREP_UAV simulator uav class
 **/

#ifndef   _GAMS_PLATFORM_VREP_UAV_H_
#define   _GAMS_PLATFORM_VREP_UAV_H_

#include "gams/platforms/Platform_Factory.h"
#include "gams/platforms/vrep/VREP_Base.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Platform_Status.h"
#include "gams/utility/GPS_Position.h"
#include "madara/knowledge_engine/Knowledge_Base.h"
#include "madara/threads/Threader.h"
#include "madara/threads/Base_Thread.h"
#include "madara/knowledge_engine/containers/Native_Double_Vector.h"

extern "C" {
#include "extApi.h"
}

#ifdef _GAMS_VREP_

namespace gams
{
  namespace platforms
  {
    class GAMS_Export VREP_UAV : public VREP_Base
    {
    public:
      /**
       * Default UAV model
       */
      const static std::string DEFAULT_UAV_MODEL;

      /**
       * Constructor
       * @param  file         model file to load
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge    knowledge base
       * @param  sensors      map of sensor names to sensor information
       * @param  platforms    map of platform names to platform information
       * @param  self         device variables that describe self state
       **/
      VREP_UAV (
        std::string model_file, 
        simxUChar is_client_side, 
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);
      
      /**
       * Gets the unique identifier of the platform. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. vrep_ant, autonomous_snake, etc.)
       **/
      virtual std::string get_id () const;

      /**
       * Gets the name of the platform
       **/
      virtual std::string get_name () const;

      /**
       * Moves the platform to a position
       * @param   position  the coordinate to move to
       * @param   epsilon   approximation value
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int move (const utility::Position & position,
        const double & epsilon = 0.1);

    protected:
      /// Move thread name
      const static std::string MOVE_THREAD_NAME;

      /**
       * Thread to move target
       **/
      class Target_Mover : public Madara::Threads::Base_Thread
      {
        public:
          /// Thread execution rate in Hz
          const static double RATE;

          /// Destination container name
          const static std::string DEST_CONTAINER_NAME;

          /**
           * Constructor
           * @param d   destination container
           * @param m   move speed for target
           **/
          Target_Mover (
            const Madara::Knowledge_Engine::Containers::Native_Double_Vector& d, 
            double m = 0);

          /**
           * main thread function
           */
          void run ();

          /**
           * set client id
           * @param c   new client id
           **/
          void set_client_id (simxInt c);

          /**
           * set node target handle in VREP
           * @param n   new target handle
           **/
          void set_node_target (simxInt n);

          /**
           * Set move speed
           * @param m   new move speed
           **/
          void set_move_speed (double m);

          /**
           * Set target position
           **/
          void set_target_pos (const utility::Position& p);

        private:
          /// client ID for VREP API
          simxInt client_id_;

          /// target node handle
          simxInt node_target_;

          /// Target move speed per second
          double move_speed_;

          /// Current target location
          utility::Position target_pos_;

          /// Destination container
          Madara::Knowledge_Engine::Containers::Native_Double_Vector destination_;
      };

      /// Thread object
      Target_Mover mover_;

      /// MADARA Threader
      Madara::Threads::Threader threader_;

      /// container for destination
      Madara::Knowledge_Engine::Containers::Native_Double_Vector thread_dest_;

      /**
       * Add model to environment
       */
      virtual void add_model_to_environment (const std::string& file, 
        const simxUChar client_side);

      /**
       * Get node target handle
       */
      virtual void get_target_handle ();

      /**
       * Set initial position for agent
       */
      virtual void set_initial_position ();
    }; // class VREP_UAV

    /**
     * A factory class for creating VREP UAV platforms
     **/
    class GAMS_Export VREP_UAV_Factory : public Platform_Factory
    {
    public:

      /**
       * Creates a VREP UAV platform.
       * @param   args      no arguments are necessary for this platform
       * @param   knowledge the knowledge base. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   platforms status inform for all known devices. This
       *                    will be set by the controller in init_vars
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       **/
      virtual Base_Platform * create (
        const Madara::Knowledge_Vector & args,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);
    };
  } // namespace platform
} // namespace gams

#endif // _GAMS_VREP_

#endif // _GAMS_PLATFORM_VREP_UAV_H_
