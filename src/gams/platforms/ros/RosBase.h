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
 * @file RosBase.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the RosBase abstract class
 **/

#ifndef   _GAMS_PLATFORM_ROS_BASE_H_
#define   _GAMS_PLATFORM_ROS_BASE_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/pose/GPSFrame.h"
#include "gams/pose/CartesianFrame.h"
#include "madara/knowledge/KnowledgeBase.h"

// ROS includes
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#ifdef _GAMS_ROS_

namespace gams
{
  namespace platforms
  {
    class GAMSExport RosBase : public BasePlatform
    {
    public:
      /**
       * Constructor
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       device variables that describe self state
       **/
      RosBase (
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Self * self);

      /**
       * Destructor
       **/
      virtual ~RosBase ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const RosBase & rhs);

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
      int move (const pose::Position & position,
        const PositionBounds &bounds = Epsilon()) override;
      
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
       * Returns the reference frame for the platform (usually GPS)
       * @return the platform's reference frame for positions
       **/
      virtual const pose::ReferenceFrame & get_frame (void) const;

      /**
       * Initialize the ros node
       * @param   n   node name
       */
      static void ros_init(const std::string& n = "robot_0");

    protected:
      /// the current frame (can theoretically be switched between options)
      pose::ReferenceFrame * frame_;

      /**
       * wait for go signal from controller
       */
      void wait_for_go () const;

      /// flag for simulated robot ready to receive instruction
      bool ready_;
    }; // class RosBase
  } // namespace platform
} // namespace gams

#endif // _GAMS_ROS_

#endif // _GAMS_PLATFORM_ROS_BASE_H_
