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
 * @file FormationFlying.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 */

#include "gams/algorithms/FormationFlying.h"

#include <cmath>
#include <string>
#include <set>

using std::set;
using std::string;
using std::stringstream;

#include "gams/utility/Position.h"
#include "gams/utility/GPSPosition.h"

#include "gams/utility/ArgumentParser.h"


namespace engine = madara::knowledge;
namespace containers = engine::containers;
namespace groups = gams::groups;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;
typedef groups::AgentVector  AgentVector;

gams::algorithms::BaseAlgorithm *
gams::algorithms::FormationFlyingFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::FormationFlyingFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string head;
    std::vector <double> offset;
    std::vector <double> destination;
    std::string group;
    std::string modifier ("default");

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 'd':
        if (i->first == "destination")
        {
          destination = i->second.to_doubles ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " %d size destination set\n", (int)destination.size ());
          break;
        }
        goto unknown;
      case 'g':
        if (i->first == "group")
        {
          group = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " setting group to %s\n", group.c_str ());
          break;
        }
        goto unknown;
      case 'h':
        if (i->first == "head")
        {
          head = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " setting formation head to %s\n", head.c_str ());
          break;
        }
        goto unknown;
      case 'm':
        if (i->first == "modifier")
        {
          modifier = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " setting modifier to %s\n", modifier.c_str ());
          break;
        }
        goto unknown;
      case 'o':
        if (i->first == "offset")
        {
          offset = i->second.to_doubles ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " %d size offset set\n", (int)offset.size ());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "target")
        {
          head = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlyingFactory:" \
            " setting formation head/target to %s\n", head.c_str ());
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationFlyingFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    // if group has not been set, use the swarm
    if (head == "" || offset.size () == 0 || destination.size () == 0 ||
      group == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationFlyingFactory::create:" \
        " Invalid args. head = %s, group = %s, " \
        " offset.size = %d, destination.size = %d. Returning null.\n",
        head.c_str (), group.c_str (),
        (int)offset.size (), (int)destination.size ());
    }
    else
    {
      result = new FormationFlying (
        head, offset, destination, group, modifier,
        knowledge, platform, sensors, self);
    }
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
gams::algorithms::FormationFlying::FormationFlying (
  const std::string & head_id,
  const std::vector<double> & offset,
  const std::vector<double> & destination,
  const std::string & group_name,
  const std::string & modifier,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : BaseAlgorithm (knowledge, platform, sensors, self), modifier_ (NONE),
    need_to_move_ (false), phi_dir_(DBL_MAX)
{
  status_.init_vars (*knowledge, "formation", self->id.to_integer ());
  status_.init_variable_values ();

  std::string my_id = "agent.";
  my_id += self->id.to_string ();

  // am i the head agent?
  head_ = (head_id == my_id);

  // set madara containers
  stringstream in_formation_str;
  in_formation_str << "formation." << head_id;
  in_formation_str << "." << my_id << ".ready";
  in_formation_.set_name (in_formation_str.str (), *knowledge);

  stringstream formation_ready_str;
  formation_ready_str << "formation." << head_id;
  formation_ready_str << ".flying";
  formation_ready_.set_name (formation_ready_str.str (), *knowledge);

  stringstream head_location_str;
  head_location_str << head_id << ".location";
  head_location_.set_name (head_location_str.str (), *knowledge, 3);

  stringstream head_destination_str;
  head_destination_str << head_id << ".destination";
  string dest_str = head_destination_str.str ();
  head_destination_.set_name(dest_str, *knowledge, 3);

  // get offset
  rho_ = offset[0];
  phi_ = offset[1];
  if (offset.size () == 3)
    z_ = offset[2];
  else
    z_ = 0.0;

  // get modifier
  if (modifier.compare ("rotate") == 0)
    modifier_ = ROTATE;

  // construct wait for in formation string
  if (head_)
  {
    // create group interface and obtain member list
    groups::GroupFactoryRepository factory (knowledge_);
    groups::GroupBase * group = factory.create (group_name);
    groups::AgentVector members;
    group->get_members (members);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::FormationFlying:" \
      " head is creating formation ready checks for %d agents\n",
      (int)members.size ());

    // construct actual string
    for (AgentVector::const_reference m : members)
    {
      if (m != my_id)
      {
        std::stringstream formation_expr;
        formation_expr << "formation." << my_id;
        formation_expr << "." << m << ".ready";

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " head is creating formation ready check for agent %s\n",
          formation_expr.str ().c_str ());

        Compiled temp;
        temp.ref = knowledge_->compile (formation_expr.str ());
        temp.agent = m;
        compiled_formation_.push_back (temp);
      }
    }

    // set destination
    destination_.latitude (destination[0]);
    destination_.longitude (destination[1]);
    if (destination.size() == 3)
      destination_.altitude (destination[2]);
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
    {
      //platform->set_move_speed (platform->get_move_speed () * 1.5);
    }
    else // head_
    {
      platform->set_move_speed (platform->get_move_speed () * 0.2);
    }
  }
  else
  {
    if (head_)
      platform->set_move_speed (platform->get_move_speed () * 0.6);
  }
}

gams::algorithms::FormationFlying::~FormationFlying ()
{
}

void
gams::algorithms::FormationFlying::operator= (
  const FormationFlying & rhs)
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
gams::algorithms::FormationFlying::analyze (void)
{
  int ret_val (UNKNOWN);

  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    // split logic by role
    if (head_)
    {
      // broadcast destination
      destination_.to_container (head_destination_);

      // head considers itself in formation when everybody else gets in formation
      if (in_formation_ == 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " head is checking if %d agents are in_formation_\n",
          (int)compiled_formation_.size ());

        int in_formation = 1;
        for (size_t i = 0; i < compiled_formation_.size (); ++i)
        {
          in_formation &= knowledge_->evaluate (compiled_formation_[i].ref).to_integer ();
          if (knowledge_->evaluate (compiled_formation_[i].ref).to_integer () == 0)
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_DETAILED,
              "gams::algorithms::FormationFlying::analyze:" \
              " agent %s not ready\n", compiled_formation_[i].agent.c_str ());
          }
          else
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_DETAILED,
              "gams::algorithms::FormationFlying::analyze:" \
              " agent %s ready\n", compiled_formation_[i].agent.c_str ());
          }
        }
        in_formation_ = in_formation;
      }
      // everybody is in formation (due to getting to this else), so inform we are ready to move
      else if (formation_ready_ == 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " head is setting formation_ready_\n");
        formation_ready_ = 1;
        ret_val = OK;
      }
    }
    else // follower
    {
      if (in_formation_ == 0) // if not yet in formation...
      {
        // calculate offset
        utility::GPSPosition start;
        start.from_container (head_location_);
        start.direction_to (get_destination (), phi_dir_);

        utility::GPSPosition location;
        location.from_container (self_->agent.location);

        // check if in formation
        if (location.approximately_equal (next_position_,
          platform_->get_accuracy ()))
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlying:" \
            " follower is setting in_formation_\n");
          in_formation_ = 1; // inform in formation
        }
      }
      else
      {
        if (formation_ready_ == 0)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlying:" \
            " follower is rebroadcasting that it's in formation\n");
          in_formation_ = 1;
        }

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " follower is moving into formation\n");

        utility::GPSPosition ref_location;
        ref_location.from_container (head_location_);
        double dist = ref_location.distance_to (get_destination ());
        // TODO: tune the movement parameter
        if (dist > platform_->get_move_speed () * 3)
        {
          // calculate offset
          utility::GPSPosition start;
          start.from_container (head_location_);
          start.direction_to (get_destination (), phi_dir_);
        }
      }
    }
  }

  return ret_val;
}

/**
 * Execute moves to next location if needed
 */
int
gams::algorithms::FormationFlying::execute (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    if (need_to_move_)
      platform_->move (next_position_);
  }

  return 0;
}

/**
 * The head agent simply moves to the destination. Other agents determine their
 * next position based on whether or not the formation is moving and the current
 * position of the head agent.
 */
int
gams::algorithms::FormationFlying::plan (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    // increment executions, only used by rotation formation for now
    ++executions_;

    need_to_move_ = false;
    if (head_)
    {
      // head only has to wait for everybody, and then move to destination
      if (formation_ready_ == 1)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " head is getting destination\n");
        next_position_ = get_destination ();
        need_to_move_ = true;
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationFlying:" \
          " head formation not ready\n");
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

        utility::GPSPosition reference;
        reference.from_container (head_location_);
        next_position_ = utility::GPSPosition::to_gps_position (
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
        utility::GPSPosition ref_location;
        ref_location.from_container (head_location_);
        utility::Position offset (rho_ * cos (angle), rho_ * sin (angle), z_);

        // hold position until everybody is ready
        if (formation_ready_ == 0)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlying:" \
            " follower moving or holding position in formation\n");
          next_position_ = utility::GPSPosition::to_gps_position (
            offset, ref_location);
        }
        else // we are moving or already at destination
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationFlying:" \
            " follower moving to destination\n");
          double dist = ref_location.distance_to (get_destination ());
          // TODO: tune the movement parameter
          if (dist > platform_->get_move_speed () * 1.5)
          {
            // predict where the reference agent will be
            dist = 1;
            utility::Position direction (
              dist * cos (phi_dir_), dist * sin (phi_dir_));
            utility::GPSPosition predicted =
              utility::GPSPosition::to_gps_position (
              direction, ref_location);
            next_position_ = utility::GPSPosition::to_gps_position (
              offset, predicted);
          }
          else // close enough, just go to final location
          {
            next_position_ = utility::GPSPosition::to_gps_position (
              offset, get_destination ());
          }
        }
        need_to_move_ = true;
      }
      }
    }
  }
  return 0;
}

bool
gams::algorithms::FormationFlying::is_head () const
{
  return head_;
}

bool
gams::algorithms::FormationFlying::is_ready () const
{
  return (formation_ready_ == 1);
}

gams::utility::GPSPosition
gams::algorithms::FormationFlying::get_destination ()
{
  utility::GPSPosition rv;
  rv.from_container(head_destination_);
  return rv;
}
