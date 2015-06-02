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
 * @file Base.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the base platform class
 **/

#ifndef   _GAMS_PLATFORM_BASE_H_
#define   _GAMS_PLATFORM_BASE_H_

#include <string>

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Platform_Status.h"
#include "gams/utility/GPS_Position.h"
#include "gams/utility/Axes.h"
#include "madara/knowledge_engine/Knowledge_Base.h"

namespace gams
{
  namespace controllers
  {
    class Base_Controller;
  }

  namespace platforms
  {
    /**
     * Possible platform statuses, as returnable by analyze ()
     **/
    enum Status
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

    class GAMS_Export Base_Platform
    {
    public:
      // allow Base controller to initialize our variables
      friend class controllers::Base_Controller;

      /**
       * Constructor
       * @param  knowledge  context containing variables and values
       * @param  sensors  map of sensor names to sensor information
       * @param  self     self referencing variables for the device
       **/
      Base_Platform (Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0);

      /**
       * Destructor
       **/
      virtual ~Base_Platform ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Base_Platform & rhs);

      /**
       * Analyzes platform information
       * @return bitmask status of the platform. @see Status.
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
      virtual void get_sensor_names (variables::Sensor_Names & sensors) const;

      /**
       * Instructs the device to return home
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int home (void);

      /**
       * Instructs the device to land
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int land (void);

      /**
       * Moves the platform to a position
       * @param   position  the coordinates to move to
       * @param   epsilon   approximation value
       * @return 1 if moving toward position, 2 if arrived, 0 if error
       **/
      virtual int move (const utility::Position & position,
        const double & epsilon = 0.1);

      /**
      * Rotates the platform an angle on a 3D axis
      * @param   axes  the coordinates to move to
      * @return 1 if currently rotating, 2 if arrived, 0 if error
      **/
      virtual int rotate (const utility::Axes & axes);

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
      void set_knowledge (Madara::Knowledge_Engine::Knowledge_Base * rhs);
      
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
       * Instructs the device to take off
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int takeoff (void);
      
      /**
       * Gets the knowledge base
       * @return the knowledge base referenced by the algorithm and platform
       **/
      Madara::Knowledge_Engine::Knowledge_Base * get_knowledge_base (void);
      
      /**
       * Gets self-referencing variables
       * @return self-referencing information like id and device attributes
       **/
      variables::Self * get_self (void);

      /**
       * Gets the available sensor information
       * @return sensor information
       **/
      variables::Sensors * get_sensors (void);

      /**
       * Gets platform status information
       * @return platform status info
       **/
      variables::Platform_Status * get_platform_status (void);

    protected:
      /// movement speed for platform in meters/second
      double move_speed_;

      /// provides access to variables and values
      Madara::Knowledge_Engine::Knowledge_Base * knowledge_;

      /// provides access to self state
      variables::Self * self_;

      /// provides access to a sensor
      variables::Sensors * sensors_;

      /// provides access to status information for this platform
      variables::Platform_Status status_;
    };

    // deprecated typdef. Please use Base_Platform instead.
    typedef  Base_Platform    Base;
  }
}

#endif // _GAMS_PLATFORM_BASE_H_
