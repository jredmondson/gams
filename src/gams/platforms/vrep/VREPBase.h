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
 * @file VREPBase.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the VREPBase simulator uav class
 **/

#ifndef   _GAMS_PLATFORM_VREP_BASE_H_
#define   _GAMS_PLATFORM_VREP_BASE_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "gams/pose/CartesianFrame.h"
#include "gams/pose/Position.h"
#include "gams/pose/Orientation.h"
#include "gams/pose/GPSFrame.h"
#include "madara/threads/Threader.h"
#include "madara/threads/BaseThread.h"
#include "madara/LockType.h"
#include "madara/knowledge/containers/Integer.h"

#include "gams/loggers/GlobalLogger.h"

extern "C" {
#include "extApi.h"
}

#ifdef _GAMS_VREP_

namespace gams
{
  namespace platforms
  {
    /**
    * A VREP platform for the base robotic system.
    **/
    class GAMSExport VREPBase : public BasePlatform
    {
    public:
      /**
       * Constructor
       * @param  file         model file to load
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       agent variables that describe self state
       **/
      VREPBase (
        std::string model_file,
        simxUChar is_client_side,
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Self * self);

      /**
       * Destructor
       **/
      virtual ~VREPBase ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const VREPBase & rhs);

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
      //virtual int move (const utility::Position & position,
        //const double & epsilon = 0.1);

      /**
       * Moves the platform to a location
       * @param   target    the coordinates to move to
       * @param   epsilon   approximation value
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      virtual int move (const pose::Position & location,
        double epsilon = 0.1);

      /**
       * Rotates the platform to a specified Orientation
       * @param   target    the coordinates to move to
       * @param   epsilon   approximation value, in radians
       * @return the status of the orient operation, @see PlatformReturnValues
       **/
      virtual int orient (const pose::Orientation & location,
        double epsilon = M_PI/32);

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

      /**
      * Method for returning the platform's current frame
      * @return frame that the platform's coordinate system is operating in
      **/
      virtual const pose::ReferenceFrame & get_frame (void) const;

      const pose::ReferenceFrame & get_vrep_frame (void) const;

    protected:
      /**
       * Add model to environment
       */
      virtual void add_model_to_environment (const std::string& file, 
        const simxUChar client_side) = 0;

      /**
      * Prep VREP for agent introduction
      * @return  if vrep and agent are ready, then true
      **/
      bool get_ready (void);

      /**
       * Get node target handle
       */
      virtual void get_target_handle () = 0;

      /**
       * Set initial position for agent
       */
      virtual void set_initial_position ();

      /**
       * Get what altitude agent should start at. Land agents might
       * override this to ensure they don't spawn in the air.
       */
      virtual double get_initial_z() const;

      /**
       * wait for go signal from controller
       */
      bool sim_is_running (void);

      /**
      * wait for signal from controller that vrep is initialized
      */
      bool vrep_is_ready (void);

      /**
      * signal for whether or not the agent has created self image in sim
      */
      bool agent_is_ready (void);

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
      pose::Pose sw_pose_;

      /**
       * Cartesian frame representing vrep coordinate system.
       **/
      pose::ReferenceFrame vrep_frame_;

      madara::knowledge::containers::Double thread_rate_;

      madara::knowledge::containers::Double thread_move_speed_;

      madara::knowledge::containers::Double max_delta_;

      madara::knowledge::containers::Double max_orient_delta_;

      /// Move thread name
      const static std::string MOVE_THREAD_NAME;

      /**
       * Thread to move target
       **/
      class TargetMover : public madara::threads::BaseThread
      {
        public:
          /**
           * Constructor
           * @param base   the VREPBase object this thread belongs to
           **/
          TargetMover (VREPBase &base);

          /**
           * main thread function
           */
          void run ();

          static const std::string NAME;
        private:
          VREPBase &base_;
      };

      friend class TargetMover;

      /// Thread object
      TargetMover *mover_;

      /// MADARA Threader
      madara::threads::Threader threader_;

      mutable MADARA_LOCK_TYPE vrep_mutex_;

    private:

      /**
       * Create the conditions for checking is_ready
       **/
      void create_ready_conditions (void);

      /// tracks if the agent itself is ready
      bool agent_is_ready_;

      /// tracks if vrep is ready for general use
      bool vrep_is_ready_;

      /// tracks if the sim has started
      bool sim_is_running_;

      /**
      * VREP model file name
      **/
      std::string model_file_;

      /**
      * Flag for whether model is on server or client side
      **/
      simxUChar is_client_side_;

      /// reference to knowledge record "begin_sim_"
      madara::knowledge::containers::Integer begin_sim_;

      /// reference to knowledge record "vrep_ready_"
      madara::knowledge::containers::Integer vrep_ready_;

      /// reference to knowledge record "S{.id}.init"
      madara::knowledge::containers::Integer agent_ready_;

      pose::Pose get_sw_pose(const pose::ReferenceFrame &frame);

      int do_move (const pose::Position & target,
                   const pose::Position & current, double max_delta);

      int do_orient (pose::Orientation target,
                     const pose::Orientation & current, double max_delta);
    }; // class VREPBase
  } // namespace platform
} // namespace gams

#include "VREPBase.inl"

#endif // _GAMS_VREP_

#endif // _GAMS_PLATFORM_VREP_BASE_H_
