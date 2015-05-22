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
 * @file VREP_Base.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the VREP_Base simulator uav class
 **/

#ifndef   _GAMS_PLATFORM_VREP_BASE_H_
#define   _GAMS_PLATFORM_VREP_BASE_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Platform.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/utility/GPS_Position.h"
#include "madara/knowledge_engine/Knowledge_Base.h"

extern "C" {
#include "extApi.h"
}

#ifdef _GAMS_VREP_

namespace gams
{
  namespace platforms
  {
    class GAMS_Export VREP_Base : public Base
    {
    public:
      /**
       * Constructor
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       device variables that describe self state
       **/
      VREP_Base (
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        variables::Sensors * sensors,
        variables::Self * self);

      /**
       * Destructor
       **/
      virtual ~VREP_Base ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const VREP_Base & rhs);

      /**
       * Polls the sensor environment for useful information
       * @return number of sensors updated/used
       **/
      virtual int sense (void);

      /**
       * Analyzes platform information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void);

      /**
       * Get the position accuracy in meters
       * @return position accuracy
       **/
      virtual double get_accuracy () const;

      /**
       * Get move speed
       **/
      virtual double get_move_speed () const;

      /**
       * Instructs the platform to land
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int land (void);

      /**
       * Moves the platform to a position
       * @param   position  the coordinate to move to
       * @param   epsilon   approximation value
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int move (const utility::Position & position,
        const double & epsilon = 0.1);
      
      /**
       * Set move speed
       * @param speed new speed in meters/loop execution
       **/
      virtual void set_move_speed (const double& speed);

      /**
       * Instructs the platform to take off
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int takeoff (void);

    protected:
      /**
       * Get float array from position
       * @param arr array to convert
       * @param pos position to store it in
       **/
      void array_to_position (const simxFloat (&arr)[3], 
        utility::Position & pos) const;

      /**
       * Converts lat/long coordinates to vrep coordinates
       * @param position    lat/long position to convert
       * @param converted   x/y coords in vrep reference frame
       **/
      void gps_to_vrep (const utility::GPS_Position & position,
        utility::Position & converted) const;

      /**
       * Get position from float array
       * @param pos position to convert
       * @param arr array to store it in
       **/
      void position_to_array (const utility::Position & pos,
        simxFloat (&arr)[3]) const;

      /**
       * Converts lat/long coordinates to vrep coordinates
       * @param position    lat/long position to convert
       * @param converted   x/y coords in vrep reference frame
       **/
      void vrep_to_gps (const utility::Position & position,
        utility::GPS_Position & converted) const;

      /**
       * Add model to environment
       */
      virtual void add_model_to_environment () = 0;

      /**
       * Get node target handle
       */
      virtual void get_target_handle () = 0;

      /**
       * Set initial position for agent
       */
      virtual void set_initial_position () const;

      /**
       * wait for go signal from controller
       */
      void wait_for_go () const;

      /// flag for drone being airborne
      bool airborne_;

      /// client id for remote API connection
      simxInt client_id_;

      /// movement speed in meters/iteration
      double move_speed_;

      /// object id for quadrotor
      simxInt node_id_;

      /// object id for quadrotor target
      simxInt node_target_;

      /// gps coordinates corresponding to (0, 0) in vrep
      utility::GPS_Position sw_position_;
    }; // class VREP_Base
  } // namespace platform
} // namespace gams

#endif // _GAMS_VREP_

#endif // _GAMS_PLATFORM_VREP_BASE_H_
