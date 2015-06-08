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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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
 * @file ROS_P3DX.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the declaration of the Stage simulated Pioneer 3D-X robot class
 **/

#ifndef   _GAMS_PLATFORM_ROS_P3DX_H_
#define   _GAMS_PLATFORM_ROS_P3DX_H_

#ifdef _GAMS_ROS_

#include "gams/platforms/ros/ROS_Base.h"

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Platform.h"
#include "gams/utility/Position.h"
#include "madara/knowledge_engine/Knowledge_Base.h"

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace gams
{
  namespace platforms
  {
    class GAMS_Export ROS_P3DX : public ROS_Base
    {
    public:
      /**
       * Constructor
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       device variables that describe self state
       **/
      ROS_P3DX (
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);

      /**
       * Destructor
       */
      virtual ~ROS_P3DX ();
      
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
      const std::string ros_namespace_;
      ros::NodeHandle node_handle_;
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
      ros::Subscriber amcl_pose_sub_;
      ros::AsyncSpinner spinner_;
      bool init_pose_set_;

      void set_initial_position(const utility::Position& p);

      void update_pose(const geometry_msgs::PoseWithCovarianceStamped& msg);
    }; // class ROS_P3DX
  } // namespace platform
} // namespace gams

#endif // _GAMS_ROS_

#endif // _GAMS_PLATFORM_ROS_P3DX_H_
