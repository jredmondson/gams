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
 * @file BasePlatform.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the base platform class
 **/

#ifndef   _GAMS_PLATFORM_BASE_H_
#define   _GAMS_PLATFORM_BASE_H_

#include <string>

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/utility/GPSPosition.h"
#include "gams/pose/ReferenceFrame.h"
#include "gams/pose/Position.h"
#include "gams/pose/Pose.h"
#include "madara/knowledge/KnowledgeBase.h"

namespace gams
{
  namespace controllers
  {
    class BaseController;
    class Multicontroller;
  }

  namespace platforms
  {
    /**
     * Possible platform statuses, as returnable by analyze ()
     **/
    enum PlatformAnalyzeStatus
    {
      UNKNOWN = 0,
      OK  = 1,
      WAITING = 2,
      DEADLOCKED = 4,
      FAILED = 8,
      MOVING = 16,
      REDUCED_SENSING_AVAILABLE = 128,
      REDUCED_MOVEMENT_AVAILABLE = 256,
      COMMUNICATION_AVAILABLE = 512,
      SENSORS_AVAILABLE = 1024,
      MOVEMENT_AVAILABLE = 2048
    };

    /**
     * Platform return values
     **/
    enum PlatformReturnValues
    {
      PLATFORM_ERROR = 0,
      PLATFORM_IN_PROGRESS = 1,
      PLATFORM_MOVING = 1,
      PLATFORM_OK = 1,
      PLATFORM_ARRIVED = 2
    };

    /// Interface for defining a bounds checker for Positions
    class GAMSExport PositionBounds {
    public:
      /// Override to return whether the current position
      /// is within the expected bounds of target
      virtual bool check_position(
          const pose::Position &current,
          const pose::Position &target) const = 0;

      virtual ~PositionBounds() = default;
    };

    /// Interface for defining a bounds checker for Orientations
    class GAMSExport OrientationBounds {
    public:
      /// Override to return whether the current orientation
      /// is within the expected bounds of target
      virtual bool check_orientation(
          const pose::Orientation &current,
          const pose::Orientation &target) const = 0;

      virtual ~OrientationBounds() = default;
    };

    /// Interface for defining a bounds checker for Poses,
    /// a combination of position and orientation checking.
    class GAMSExport PoseBounds :
      public PositionBounds, public OrientationBounds {};

    /// A simple bounds checker which tests whether the
    /// current position is within the given number of
    /// meters of the expected position, and whether the
    /// difference in angles is within the given number
    /// of radians.
    class GAMSExport Epsilon : public PoseBounds {
    private:
      double dist_ = 0.1, radians_ = M_PI/16;
    public:
      /// Use default values for position and angle tolerance.
      Epsilon() {}

      /// Use default value for angle tolerance.
      ///
      /// @param dist the position tolerance (in meters)
      Epsilon(double dist)
        : dist_(dist) {}

      /// Use specified tolerances
      ///
      /// @param dist the position tolerance (in meters)
      /// @param radians the angle tolerance (in radians)
      Epsilon(double dist, double radians)
        : dist_(dist), radians_(radians) {}

      /// Use specified tolerances, with custom angle units
      ///
      /// @param dist the position tolerance (in meters)
      /// @param angle the angle tolerance (in units given)
      /// @param u an angle units object (such as pose::radians,
      ///      pose::degrees, or pose::revolutions)
      //
      /// @tparam AngleUnits the units type for angle (inferred from u)
      template<typename AngleUnits>
      Epsilon(double dist, double angle, AngleUnits u)
        : dist_(dist), radians_(u.to_radians(angle)) {}

      bool check_position(
          const pose::Position &current,
          const pose::Position &target) const override {
        return current.distance_to(target) <= dist_;
      }

      bool check_orientation(
          const pose::Orientation &current,
          const pose::Orientation &target) const override {
        return fabs(current.angle_to(target)) <= radians_;
      }
    };

    /**
    * The base platform for all platforms to use
    **/
    class GAMSExport BasePlatform
    {
    public:
      // allow Base controller to initialize our variables
      friend class controllers::BaseController;
      friend class controllers::Multicontroller;

      /**
       * Constructor
       * @param  knowledge  context containing variables and values
       * @param  sensors  map of sensor names to sensor information
       * @param  self     self referencing variables for the agent
       **/
      BasePlatform (
        madara::knowledge::KnowledgeBase * knowledge = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      virtual ~BasePlatform ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const BasePlatform & rhs);

      /**
       * Analyzes platform information
       * @return bitmask status of the platform. @see PlatformAnalyzeStatus.
       **/
      virtual int analyze (void) = 0;

      /**
       * Gets the position accuracy in meters
       * @return position accuracy
       **/
      virtual double get_accuracy (void) const;

      /**
       * Gets GPS position
       * @return GPS location of platform
       */
      utility::Position * get_position ();

      /**
       * Gets Location of platform, within its parent frame
       * @return Location of platform
       */
      pose::Position get_location () const;

      /**
       * Gets Orientation of platform, within its parent frame
       * @return Location of platform
       */
      pose::Orientation get_orientation () const;

      /**
       * Gets Pose of platform, within its parent frame
       * @return Location of platform
       */
      pose::Pose get_pose () const;

      /**
       * Gets the name of the platform
       **/
      virtual std::string get_name () const = 0;

      /**
       * Gets the unique identifier of the platform. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. vrep_ant, autonomous_snake, etc.)
       **/
      virtual std::string get_id () const = 0;

      /**
       * Gets sensor radius
       * @return minimum radius of all available sensors for this platform
       */
      virtual double get_min_sensor_range () const;

      /**
       * Gets move speed
       **/
      virtual double get_move_speed () const;

      /**
       * Gets a sensor
       * @param name  identifier of sensor to get
       * @return Sensor object
       */
      virtual const variables::Sensor & get_sensor (const std::string& name) const;

      /**
       * Fills a list of sensor names with sensors available on the platform
       * @param  sensors   list of sensors to fill
       **/
      virtual void get_sensor_names (variables::SensorNames & sensors) const;

      /**
       * Instructs the agent to return home
       * @return the status of the home operation, @see PlatformReturnValues
       **/
      virtual int home (void);

      /**
       * Instructs the agent to land
       * @return the status of the land operation, @see PlatformReturnValues
       **/
      virtual int land (void);

      /**
       * Moves the platform to a location
       * @param   location    the coordinates to move to
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      virtual int move (const pose::Position & target) {
        return move (target, Epsilon());
      }

      /**
       * Moves the platform to a location
       * @param   location    the coordinates to move to
       * @param   bounds      object to compute if platform has arrived
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      virtual int move (const pose::Position & target,
        const PositionBounds &bounds);

      /**
       * Moves the platform to a location
       * @param   location    the coordinates to move to
       * @param   epsilon     approximation value
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      int move (const pose::Position & target, double epsilon) {
        return move (target, Epsilon(epsilon));
      }

      /**
       * Rotates the platform to match a given angle
       * @param   target    the orientation to move to
       * @return the status of the orient, @see PlatformReturnValues
       **/
      virtual int orient (const pose::Orientation & target) {
        return orient (target, Epsilon());
      }

      /**
       * Rotates the platform to match a given angle
       * @param   target    the orientation to move to
       * @param   bounds      object to compute if platform has arrived
       * @return the status of the orient, @see PlatformReturnValues
       **/
      virtual int orient (const pose::Orientation & target,
        const OrientationBounds &bounds);

      /**
       * Rotates the platform to match a given angle
       * @param   target    the orientation to move to
       * @param   epsilon   approximation value
       * @return the status of the orient, @see PlatformReturnValues
       **/
      int orient (const pose::Orientation & target, double epsilon) {
        return orient(target, Epsilon (epsilon));
      }

      /**
       * Moves the platform to a pose (location and orientation)
       *
       * This default implementation calls move and orient with the
       * Location and Orientation portions of the target Pose. The return value
       * is composed as follows: if either call returns ERROR (0), this call
       * also returns ERROR (0). Otherwise, if BOTH calls return ARRIVED (2),
       * this call also returns ARRIVED (2). Otherwise, this call returns
       * MOVING (1)
       *
       * Overrides might function differently.
       *
       * @param   target        the coordinates to move to
       * @return the status of the operation, @see PlatformReturnValues
       **/
      virtual int pose (const pose::Pose & target) {
        return pose (target, Epsilon());
      }

      /**
       * Moves the platform to a pose (location and orientation)
       *
       * This default implementation calls move and orient with the
       * Location and Orientation portions of the target Pose. The return value
       * is composed as follows: if either call returns ERROR (0), this call
       * also returns ERROR (0). Otherwise, if BOTH calls return ARRIVED (2),
       * this call also returns ARRIVED (2). Otherwise, this call returns
       * MOVING (1)
       *
       * Overrides might function differently.
       *
       * @param   target        the coordinates to move to
       * @param   bounds      object to compute if platform has arrived
       * @return the status of the operation, @see PlatformReturnValues
       **/
      virtual int pose (const pose::Pose & target, const PoseBounds &bounds);

      /**
       * Moves the platform to a pose (location and orientation)
       *
       * This default implementation calls move and orient with the
       * Location and Orientation portions of the target Pose. The return value
       * is composed as follows: if either call returns ERROR (0), this call
       * also returns ERROR (0). Otherwise, if BOTH calls return ARRIVED (2),
       * this call also returns ARRIVED (2). Otherwise, this call returns
       * MOVING (1)
       *
       * Overrides might function differently.
       *
       * @param   target        the coordinates to move to
       * @param   loc_epsilon   approximation value for the location
       * @param   rot_epsilon   approximation value for the orientation
       * @return the status of the operation, @see PlatformReturnValues
       **/
      int pose (const pose::Pose & target,
        double loc_epsilon, double rot_epsilon = M_PI/16) {
        return pose(target, Epsilon(loc_epsilon, rot_epsilon));
      }

      /**
       * Pauses movement, keeps source and dest at current values
       **/
      virtual void pause_move (void);

      /**
       * Polls the sensor environment for useful information
       * @return number of sensors updated/used
       **/
      virtual int sense (void) = 0;

      /**
       * Sets the knowledge base to use for the platform
       * @param  rhs  the new knowledge base to use
       **/
      void set_knowledge (madara::knowledge::KnowledgeBase * rhs);

      /**
       * Set move speed
       * @param speed new speed in meters/second
       **/
      virtual void set_move_speed (const double& speed);

      /**
       * Sets the map of sensor names to sensor information
       * @param  sensors      map of sensor names to sensor information
       **/
      virtual void set_sensors (variables::Sensors * sensors);

      /**
       * Stops movement, resetting source and dest to current location
       **/
      virtual void stop_move (void);

      /**
      * Resumes movement status flags
      **/
      virtual void resume_move (void);

      /**
      * Resumes orientation status flags
      **/
      virtual void resume_orientation (void);

      /**
       * Stops orientation, resetting source and dest angles to current angle
       **/
      virtual void stop_orientation (void);

      /**
       * Instructs the agent to take off
       * @return the status of the takeoff, @see PlatformReturnValues
       **/
      virtual int takeoff (void);

      /**
       * Gets the knowledge base
       * @return the knowledge base referenced by the algorithm and platform
       **/
      madara::knowledge::KnowledgeBase * get_knowledge_base (void) const;

      /**
       * Gets self-referencing variables
       * @return self-referencing information like id and agent attributes
       **/
      variables::Self * get_self (void) const;

      /**
       * Gets the available sensor information
       * @return sensor information
       **/
      variables::Sensors * get_sensors (void) const;

      /**
       * Gets platform status information
       * @return platform status info
       **/
      variables::PlatformStatus * get_platform_status (void);

      /**
       * Gets platform status information (const version)
       * @return platform status info
       **/
      const variables::PlatformStatus * get_platform_status (void) const;

      /**
       * Method for returning the platform's current frame
       *
       * By default, returns pose::default_frame()
       *
       * @return frame that the platform's coordinate system is operating in
       **/
      virtual const pose::ReferenceFrame & get_frame (void) const;

    protected:
      /// movement speed for platform in meters/second
      double move_speed_;

      /// provides access to variables and values
      madara::knowledge::KnowledgeBase * knowledge_;

      /// provides access to self state
      variables::Self * self_;

      /// provides access to a sensor
      variables::Sensors * sensors_;

      /// provides access to status information for this platform
      variables::PlatformStatus status_;

      /// the reference frame this platform operates within
      //static pose::GPSFrame frame_;
    };

    // deprecated typdef. Please use BasePlatform instead.
    typedef  BasePlatform    Base;
  }
}

#include "BasePlatform.inl"

#endif // _GAMS_PLATFORM_BASE_H_
