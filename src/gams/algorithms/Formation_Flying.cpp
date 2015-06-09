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
 * @file Formation_Flying.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 */

#include "gams/algorithms/Formation_Flying.h"

#include <cmath>
#include <string>
#include <set>

using std::set;
using std::string;
using std::stringstream;

#include "gams/utility/Position.h"
#include "gams/utility/GPS_Position.h"


gams::algorithms::Base_Algorithm *
gams::algorithms::Formation_Flying_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  Base_Algorithm * result (0);

  // set default parameters
  Madara::Knowledge_Record modifier ("default");
  
  if (knowledge && sensors && platform && self && args.size () >= 4)
  {
    if (args.size () == 5)
      modifier = args[4];

    result = new Formation_Flying (
      args[0] /* target */,
      args[1] /* offset */,
      args[2] /* destination */,
      args[3] /* members */,
      modifier /* for rotation */,
      knowledge, platform, sensors, self);
  }

  return result;
}


/**
 * Formation flying has several parameters. The head of the formation is what 
 * the other agents key off of to determine their location. Offset is the 
 * agent's specified location (in cylindrical coordinates) relative to the head
 * agent. Destination is the final position for the head agent. Members is the 
 * number of members in the formation, used to synchronize starting. Modifier
 * is either NONE or ROTATE (rotate the formation).
 */
gams::algorithms::Formation_Flying::Formation_Flying (
  const Madara::Knowledge_Record & head_id,
  const Madara::Knowledge_Record & offset,
  const Madara::Knowledge_Record & destination,
  const Madara::Knowledge_Record & members,
  const Madara::Knowledge_Record & modifier,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base_Algorithm (knowledge, platform, sensors, self), modifier_ (NONE),
    need_to_move_ (false), phi_dir_(DBL_MAX)
{
  status_.init_vars (*knowledge, "formation");

  // am i the head agent?
  head_ = (head_id.to_integer () == self->id.to_integer ());

  // set madara containers
  stringstream in_formation_str;
  in_formation_str << "formation." << head_id.to_integer ();
  in_formation_str << "." << *(self->id) << ".ready";
  in_formation_.set_name (in_formation_str.str (), *knowledge);

  stringstream formation_ready_str;
  formation_ready_str << "formation." << head_id.to_integer ();
  formation_ready_str << ".flying";
  formation_ready_.set_name (formation_ready_str.str (), *knowledge);

  stringstream head_location_str;
  head_location_str << "device." << head_id.to_integer () << ".location";
  head_location_.set_name (head_location_str.str (), *knowledge, 3);

  stringstream head_destination_str;
  head_destination_str << "device." << head_id.to_integer () << ".destination";
  string dest_str = head_destination_str.str ();
  head_destination_.set_name(dest_str, *knowledge, 3);

  // parse offset
  if (!head_)
    sscanf (offset.to_string ().c_str (), "%lf,%lf,%lf", &rho_, &phi_, &z_);

  // parse modifier
  string mod = modifier.to_string ();
  if (mod == "rotate")
  {
    modifier_ = ROTATE;
  }

  // construct wait for in formation string
  if (head_)
  {
    // parse members
    // TODO: clean this up
    char* mem_string = new char[members.to_string ().length () + 1];
    const char* idx = mem_string;
    strcpy (mem_string, members.to_string ().c_str ());
    int num_members, member;
    sscanf (idx, "%d,%*s", &num_members);
    idx = strchr (idx, ',') + 1;
    set<int> members;
    for (int i = 0; i < num_members - 1; ++i)
    {
      sscanf (idx, "%d,%*s", &member);
      if (member != self->id.to_integer ())
        members.insert (member);
      idx = strchr (idx, ',') + 1;
    }
    sscanf (idx, "%d", &member);
    if (member != self->id.to_integer ())
      members.insert (member);
    delete [] mem_string;

    // construct actual string
    std::stringstream formation_expr;
    set<int>::iterator it = members.begin ();
    formation_expr << "formation." << self->id.to_integer ();
    formation_expr << "." << *it << ".ready ";
    ++it;
    for (; it != members.end (); ++it)
    {
      formation_expr << " && formation." << head_id.to_integer ();
      formation_expr << "." << *it << ".ready ";
    }
    compiled_formation_ = knowledge_->compile (formation_expr.str ());

    // set destination
    double lat, lon, alt;
    sscanf (destination.to_string ().c_str (), "%lf,%lf,%lf", &lat, &lon, &alt);
    destination_.latitude (lat);
    destination_.longitude (lon);
    destination_.altitude (alt);
    destination_.to_container (head_destination_);
  }

  /**
   * These values are used because they were found to produce simulations
   * that looked good in VREP. These will be parameterized when we perform real
   * world experiments, but for now will remain hardcoded.
   */
  // update speed if necessary
  if (modifier_ == ROTATE)
  {
    if (!head_)
      platform->set_move_speed (platform->get_move_speed () * 1.5);
    else // head_
      platform->set_move_speed (platform->get_move_speed () * 0.2);
  }
  else
  {
    if (head_)
      platform->set_move_speed (platform->get_move_speed () * 0.6);
  }
}

gams::algorithms::Formation_Flying::~Formation_Flying ()
{
}

void
gams::algorithms::Formation_Flying::operator= (
  const Formation_Flying & rhs)
{
  if (this != &rhs)
  {
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
  }
}

/**
 * The analyze function determines if an agent is in formation and ready to 
 * begin moving and changes behavior based on distance from destination
 */
int
gams::algorithms::Formation_Flying::analyze (void)
{
  int ret_val (UNKNOWN);

  // split logic by role
  if (head_)
  {
    // broadcast destination
    destination_.to_container (head_destination_);

    // head considers itself in formation when everybody else gets in formation
    if (in_formation_ == 0)
    {
      in_formation_ = knowledge_->evaluate (compiled_formation_).to_integer ();
    }
    // everybody is in formation, so inform we are ready to move
    else if (formation_ready_ == 0)
    {
      formation_ready_ = 1;
      ret_val = OK;
    }
  }
  else // follower
  {
    if (in_formation_ == 0) // if not yet in formation...
    {
      // calculate offset
      utility::GPS_Position start;
      start.from_container (head_location_);
      start.direction_to (get_destination (), phi_dir_);

      utility::GPS_Position location;
      location.from_container (self_->device.location);

      // check if in formation
      if (location.approximately_equal (next_position_,
        platform_->get_accuracy ()))
      {
        in_formation_ = 1; // inform in formation
      }
    }
    else
    {
      utility::GPS_Position ref_location;
      ref_location.from_container (head_location_);
      double dist = ref_location.distance_to (get_destination ());
      // TODO: tune the movement parameter
      if (dist > platform_->get_move_speed () * 3)
      {
        // calculate offset
        utility::GPS_Position start;
        start.from_container (head_location_);
        start.direction_to (get_destination (), phi_dir_);
      }
    }
  }
  return ret_val;
}

/**
 * Execute moves to next location if needed
 */
int
gams::algorithms::Formation_Flying::execute (void)
{
  if (need_to_move_)
    platform_->move (next_position_);
  return 0;
}

/**
 * The head agent simply moves to the destination. Other agents determine their
 * next position based on whether or not the formation is moving and the current
 * position of the head agent.
 */
int
gams::algorithms::Formation_Flying::plan (void)
{
  // increment executions, only used by rotation formation for now
  ++executions_;

  need_to_move_ = false;
  if (head_)
  {
    // head only has to wait for everybody, and then move to destination
    if (formation_ready_ == 1)
    {
      next_position_ = get_destination ();
      need_to_move_ = true;
    }
  }
  else // !head_
  {
    switch (modifier_)
    {
      /**
       * Rotation formation keys off of head location at all times
       */
      case ROTATE:
      {
        const double OMEGA = M_PI / 30;
        double angle = -phi_ + phi_dir_ + executions_ * OMEGA;
        utility::Position offset (rho_ * cos (angle), rho_ * sin (angle), z_);

        utility::GPS_Position reference;
        reference.from_container (head_location_);
        next_position_ = utility::GPS_Position::to_gps_position (
          offset, reference);
        
        need_to_move_ = true;

        break;
      }

      /**
       * Default formation sticks with head, until close to destination
       */
      default: // case NONE
      {
        // calculate formation location
        double angle = phi_ + phi_dir_;
        utility::GPS_Position ref_location;
        ref_location.from_container (head_location_);
        utility::Position offset (rho_ * cos (angle), rho_ * sin (angle), z_);

        // hold position until everybody is ready
        if (formation_ready_ == 0)
        {
          next_position_ = utility::GPS_Position::to_gps_position (
            offset, ref_location);
        }
        else // we are moving or already at destination
        {
          double dist = ref_location.distance_to (get_destination ());
          // TODO: tune the movement parameter
          if (dist > platform_->get_move_speed ())
          {
            // predict where the reference device will be
            dist = platform_->get_move_speed () * 1.5;
            utility::Position direction (
              dist * cos (phi_dir_), dist * sin (phi_dir_));
            utility::GPS_Position predicted =
              utility::GPS_Position::to_gps_position (
              direction, ref_location);
            next_position_ = utility::GPS_Position::to_gps_position (
              offset, predicted);
          }
          else // close enough, just go to final location
          {
            next_position_ = utility::GPS_Position::to_gps_position (
              offset, get_destination ());
          }
        }
        need_to_move_ = true;
      }
    }
  }
  return 0;
}

bool
gams::algorithms::Formation_Flying::is_head () const
{
  return head_;
}

gams::utility::GPS_Position
gams::algorithms::Formation_Flying::get_destination ()
{
  utility::GPS_Position rv;
  rv.from_container(head_destination_);
  return rv;
}
