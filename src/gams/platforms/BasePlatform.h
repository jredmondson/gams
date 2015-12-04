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
#include "gams/utility/Axes.h"
#include "gams/utility/GPSFrame.h"
#include "gams/utility/Location.h"
#include "gams/utility/Pose.h"
#include "madara/knowledge/KnowledgeBase.h"

namespace gams
{
  namespace controllers
  {
    class BaseController;
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
      PLATFORM_ARRIVED = 2
    };

    /**
    * The base platform for all platforms to use
    **/
    class GAMSExport BasePlatform
    {
    public:
      // allow Base controller to initialize our variables
      friend class controllers::BaseController;

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
      utility::Location get_location () const;

      /**
       * Gets Rotation of platform, within its parent frame
       * @return Location of platform
       */
      utility::Rotation get_rotation () const;

      /**
       * Gets Pose of platform, within its parent frame
       * @return Location of platform
       */
      utility::Pose get_pose () const;

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
       * Moves the platform to a position
       * @param   position  the coordinates to move to
       * @param   epsilon   approximation value
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      virtual int move (const utility::Position & position,
        const double & epsilon = 0.1);

      /**
       * Moves the platform to a location
       * @param   target    the coordinates to move to
       * @param   epsilon   approximation value
       * @return the status of the move operation, @see PlatformReturnValues
       **/
      virtual int move (const utility::Location & location,
        double epsilon = 0.1);

      /**
      * Rotates the platform by an angle on a 3D axis
      * @param   axes  the coordinates to move to
      * @return the status of the rotate, @see PlatformReturnValues
      **/
      virtual int rotate (const utility::Axes & axes);

      /**
       * Rotates the platform to match a given angle
       * @param   target    the rotation to move to
       * @param   epsilon   approximation value
       * @return the status of the rotate, @see PlatformReturnValues
       **/
      virtual int rotate (const utility::Rotation & target,
        double epsilon = M_PI/16);

      /**
       * Moves the platform to a pose (location and rotation)
       *
       * This default implementation calls move and rotate with the
       * Location and Rotation portions of the target Pose. The return value
       * is composed as follows: if either call returns ERROR (0), this call
       * also returns ERROR (0). Otherwise, if BOTH calls return ARRIVED (2),
       * this call also returns ARRIVED (2). Otherwise, this call returns
       * MOVING (1)
       *
       * Overrides might function differently.
       *
       * @param   target        the coordinates to move to
       * @param   loc_epsilon   approximation value for the location
       * @param   rot_epsilon   approximation value for the rotation
       * @return the status of the operation, @see PlatformReturnValues
       **/
      virtual int pose (const utility::Pose & target,
        double loc_epsilon = 0.1, double rot_epsilon = M_PI/16);

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

      static const utility::ReferenceFrame &get_frame (void);

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
      static utility::GPSFrame frame_;
    };

    // deprecated typdef. Please use BasePlatform instead.
    typedef  BasePlatform    Base;
  }
}

#include "BasePlatform.inl"

#endif // _GAMS_PLATFORM_BASE_H_
