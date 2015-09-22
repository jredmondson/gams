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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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
 * @file Formation_Sync.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Definitions for Formation_Sync class
 * - The head agent runs some area coverage algorithm
 * - Followers perform formation flying around the head agent
 **/

#include "gams/algorithms/Formation_Sync.h"
#include "gams/algorithms/Controller_Algorithm_Factory.h"
#include "madara/knowledge_engine/containers/String_Vector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "gams/algorithms/Algorithm_Factory.h"

namespace engine = Madara::Knowledge_Engine;
namespace containers = engine::Containers;

typedef Madara::Knowledge_Record::Integer  Integer;

const int pyramid_cols[] = {
  0,                                 // 0 processes have 0 cols
  1,                                 // 1 process has 1 cols
  3, 3,                              // 2-3 processes have 3 cols
  5, 5, 5,                           // 4-6 processes have 5 cols
  7, 7, 7, 7,                        // 7-10 processes have 7 cols
  9, 9, 9, 9, 9,                     // 11-15 processes have 9 cols
  11, 11, 11, 11, 11, 11,            // 16-21 processes have 11
  13, 13, 13, 13, 13, 13, 13,        // 13 columns
  15, 15, 15, 15, 15, 15, 15, 15,    // 15 columns
  17, 17, 17, 17, 17, 17, 17, 17, 17 // 17 columns
};

gams::algorithms::Base_Algorithm *
gams::algorithms::Formation_Sync_Factory::create (
const Madara::Knowledge_Vector & args,
Madara::Knowledge_Engine::Knowledge_Base * knowledge,
platforms::Base_Platform * platform,
variables::Sensors * sensors,
variables::Self * self,
variables::Devices * devices)
{
  Base_Algorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync_Factory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    utility::GPS_Position start;
    utility::GPS_Position end;
    std::vector <std::string> members;

    int formation_type = Formation_Sync::LINE;
    double buffer = 5.0;
    std::string group = "";
    std::string barrier = "barrier.formation_sync";

    for (size_t i = 0; i < args.size (); ++i)
    {
      // if a start position is being specified
      if (args[i] == "start" && i + 1 < args.size ())
      {
        std::vector <double> coords (args[i + 1].to_doubles ());

        if (coords.size () >= 2)
        {
          start.x = coords[0];
          start.y = coords[1];
        }

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::Formation_Sync_Factory:" \
          " setting start to %s\n", start.to_string ().c_str ());

        ++i;
      }
      else if (args[i] == "end" && i + 1 < args.size ())
      {
        std::vector <double> coords (args[i + 1].to_doubles ());

        if (coords.size () >= 2)
        {
          end.x = coords[0];
          end.y = coords[1];
        }

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::Formation_Sync_Factory:" \
          " setting end to %s\n", end.to_string ().c_str ());

        ++i;
      }
      else if (args[i] == "group" && i + 1 < args.size ())
      {
        group = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::Formation_Sync_Factory:" \
          " setting group to %s\n", group.c_str ());

        std::string members_list_name = "group." + group + ".members";

        containers::String_Vector member_list (members_list_name, *knowledge);

        member_list.copy_to (members);

        ++i;
      }
      else if (args[i] == "buffer" && i + 1 < args.size ())
      {
        buffer = args[i + 1].to_double ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::Formation_Sync_Factory:" \
          " setting buffer to %f\n", buffer);

        ++i;
      }
      else if (args[i] == "barrier" && i + 1 < args.size ())
      {
        barrier = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::Formation_Sync_Factory:" \
          " setting barrier to %s\n", barrier.c_str ());

        ++i;
      }
      else if (args[i] == "formation" && i + 1 < args.size ())
      {
        std::string formation_str = args[i + 1].to_string ();

        Madara::Utility::upper (formation_str);

        if (formation_str == "PYRAMID")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to PYRAMID\n");

          formation_type = Formation_Sync::PYRAMID;
        }
        else if (formation_str == "TRIANGLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to TRIANGLE\n");

          formation_type = Formation_Sync::TRIANGLE;
        }
        else if (formation_str == "RECTANGLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to RECTANGLE\n");

          formation_type = Formation_Sync::RECTANGLE;
        }
        else if (formation_str == "CIRCLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to CIRCLE\n");

          formation_type = Formation_Sync::CIRCLE;
        }
        else if (formation_str == "LINE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to LINE\n");

          formation_type = Formation_Sync::LINE;
        }
        else if (args[i + 1].is_integer_type ())
        {
          formation_type = (int)args[i + 1].to_integer ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::Formation_Sync_Factory:" \
            " setting formation to %d\n", formation_type);
        }

        ++i;
      }
    }

    // if group has not been set, use the swarm
    if (group == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::Formation_Sync::constructor:" \
        " No group specified. Using swarm.\n");

      Integer processes = (Integer)devices->size ();

      for (Integer i = 0; i < processes; ++i)
      {
        Madara::Knowledge_Record temp ("device.");
        temp += i;
        members.push_back (temp.to_string ());
      }
    }

    result = new Formation_Sync (start, end, members, buffer,
      formation_type, barrier,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Formation_Sync::Formation_Sync (
  utility::GPS_Position & start,
  utility::GPS_Position & end,
  const std::vector<std::string> & members,
  double buffer,
  int formation,
  std::string barrier_name,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  Base_Algorithm (knowledge, platform, sensors, self), start_ (start),
  end_ (end), members_ (members), buffer_ (buffer), formation_ (formation)
{
  status_.init_vars (*knowledge, "formation_sync", self->id.to_integer ());
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Creating algorithm with args: " \
    " start=%s, end=%s, buffer=%.2f, formation=%d\n",
    start.to_string ().c_str (), end.to_string ().c_str (), buffer, formation);

  generate_plan (formation);

  if (position_ >= 0)
  {
    barrier_.set_name (barrier_name, *knowledge, position_, (int)members.size ());
    barrier_.set (0);
    barrier_.next ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " device.%d does not have a position in group algorithm." \
      " Unable to participate in barrier.\n",
      (int)self_->id.to_integer ());

  }
}

void
gams::algorithms::Formation_Sync::generate_plan (int formation)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Generating plan\n");

  Madara::Knowledge_Record temp ("device.");
  temp += self_->id.to_string ();

  position_ = this->get_position_in_member_list (temp.to_string (), members_);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MINOR,
    "gams::algorithms::Formation_Sync::constructor:" \
    " %s is position %d in member list\n",
    temp.to_string ().c_str (), position_);

  /**
   * the offset in the line has an open space between each process
   * [0] [ ] [1] [ ] [2] [ ] [3] [ ] [4] [ ] [5] ...
   * initial position will be ref + position * buffer
   **/

  /**
   * First step, figure out how many steps we need. Distances
   * will contain the latitude (x) and longitude (y) differences
   * between the points in meters.
   **/
  utility::Position distances = end_.to_position (start_);

  /**
   * the number of moves is sum of the the horizontal and
   * vertical distances divided by the buffer
   **/
  int x_moves = (int)(std::abs (distances.x / buffer_)) + 1;
  int y_moves = (int)(std::abs (distances.y / buffer_)) + 1;
  int total_moves = x_moves + y_moves;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Formation will move %.3f m lat, %.3f m long in %d moves\n",
    distances.x, distances.y, total_moves);

  // the type of movement will be based on sign of distance
  double latitude_move = distances.x < 0 ? -buffer_ : buffer_;
  double longitude_move = distances.y < 0 ? -buffer_ : buffer_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Will execute %.3f m in %d latitude moves and then" \
    " %.3f m in %d longitude moves\n",
    latitude_move, x_moves, longitude_move, y_moves);

  // a cartesian movement offset
  utility::Position movement;

  // the initial position for this specific device
  utility::GPS_Position init;
  utility::GPS_Position position_end;

  if (formation == TRIANGLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " Formation type is TRIANGLE\n");

    /**
    * the offset in the line has an open space between each process
    * [0] [ ] [1] [ ] [2] [ ] n / 2 agents = row
    * [ ] [3] [ ] [4] row - 1 agents
    * [5] row - 2 agents
    * initial position will be ref + position * buffer
    **/

    // first row
    if (position_ <= members_.size () / 2)
    {
      movement.x = position_ * latitude_move * 2;
      movement.y = 0;
    }
    else
    {
      bool position_found = false;
      int row_length = (int)members_.size () / 4;
      int last_position = (int)members_.size () / 2;
      for (int row = 1; !position_found; ++row, last_position += row_length, row_length /= 2)
      {
        if (row_length < 1)
          row_length = 1;

        if (position_ <= last_position + row_length)
        {
          int column = position_ - last_position - 1;
          position_found = true;

          // stagger the rows for a seamless buffer space for neighbor rows
          if (row % 2 == 0)
          {
            movement.x = column * latitude_move * 2;
          }
          else
          {
            movement.x = latitude_move + column * latitude_move * 2;
          }
          movement.y = row * longitude_move;
        }
      }
    }

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end = end_.to_gps_position (
      movement, end_);

  }
  else if (formation == PYRAMID)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " Formation type is PYRAMID\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end.x = end_.x + position_ * latitude_move * 2;
    position_end.y = end_.y;

  }
  else if (formation == RECTANGLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " Formation type is RECTANGLE\n");

    /**
    * the offset in the line has an open space between each process
    * [0] [ ] [1] [ ] 
    * [ ] [2] [ ] [3] 
    * [4] [ ] [5] [ ]
    * initial position will be ref + position * buffer
    **/

    double num_rows = std::sqrt ((double)members_.size ());
    int column (0), row (0);

    column = position_ % (int)num_rows;
    row = position_ / (int)num_rows;

    // the initial position where the first two moves will be for this device
    if (row % 2 == 0)
    {
      movement.x = column * latitude_move * 2;
    }
    else
    {
      movement.x = latitude_move + column * latitude_move * 2;
    }
    movement.y = row * longitude_move;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end.x = end_.x + position_ * latitude_move * 2;
    position_end.y = end_.y;

  }
  else if (formation == CIRCLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " Formation type is CIRCLE\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end.x = end_.x + position_ * latitude_move * 2;
    position_end.y = end_.y;

  }
  // default is LINE
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::constructor:" \
      " Formation type is LINE\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end.x = end_.x + position_ * latitude_move * 2;
    position_end.y = end_.y;
  }

  utility::GPS_Position last (init);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Position[%d] will begin at %s\n",
    position_, last.to_string ().c_str ());

  // first two barriered moves are the initial position
  plan_.push_back (last);
  plan_.push_back (last);

  movement.x = latitude_move;
  movement.y = 0;

  // move latitude until done
  for (int i = 0; i < x_moves; ++i)
  {
    last = last.to_gps_position (
      movement, last);
    plan_.push_back (last);
  }

  // push last latitudinal position onto plan
  //last.x = position_end.x;
  //plan_.push_back (last);

  // keep track of the pivot point
  move_pivot_ = x_moves + 2;

  // adjust longitudinally
  movement.x = 0;
  movement.y = longitude_move;

  // move latitude until done
  for (int i = 0; i < y_moves; ++i)
  {
    last = last.to_gps_position (
      movement, last);
    plan_.push_back (last);
  }

  // push last latitudinal position onto plan
  //last.y = position_end.y;
  //plan_.push_back (last);

  std::stringstream full_plan_description;

  for (size_t i = 0; i < plan_.size (); ++i)
  {
    full_plan_description << "  [" << i << "]: ";
    full_plan_description << plan_[i].to_string () << "\n";
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::constructor:" \
    " Generated the following plan:\n%s",
    full_plan_description.str ().c_str ());

}

gams::utility::GPS_Position
gams::algorithms::Formation_Sync::generate_position (utility::GPS_Position reference,
double angle, double distance)
{
  // compute the offset position
  utility::Position offset (
    distance * cos (angle), distance * sin (angle), 0);

  // return the new GPS position from the reference position
  return utility::GPS_Position::to_gps_position (
    offset, reference);
}

int
gams::algorithms::Formation_Sync::get_position_in_member_list (
std::string id,
std::vector <std::string> & member_list)
{
  int result = -1;

  for (size_t i = 0; i < member_list.size (); ++i)
  {
    if (member_list[i] == id)
    {
      result = (int)i;
      break;
    }
  }

  return result;
}

gams::algorithms::Formation_Sync::~Formation_Sync ()
{
}

void
gams::algorithms::Formation_Sync::operator= (const Formation_Sync & rhs)
{
  if (this != &rhs)
  {
    this->Base_Algorithm::operator= (rhs);

    start_ = rhs.start_;
    end_ = rhs.end_;
    members_ = rhs.members_;
    buffer_ = rhs.buffer_;
  }
}

int
gams::algorithms::Formation_Sync::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::analyze:" \
    " entering analyze method\n");

  if (position_ >= 0)
  {
    int round = barrier_.get_round ();

    barrier_.modify ();

    if (round < (int)plan_.size () && barrier_.is_done ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::Formation_Sync::analyze:" \
        " %d: Round %d of %d: Proceeding to next barrier round\n",
        position_, round, (int)plan_.size ());

      utility::GPS_Position current;
      current.from_container (self_->device.location);

      if (plan_[round].approximately_equal (current, platform_->get_accuracy ()))
      {
        barrier_.next ();
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::Formation_Sync::analyze:" \
        " %d: Round %d of %d: NOT proceeding to next barrier round\n",
        position_, round, (int)plan_.size ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::analyze:" \
      " device.%d does not have a position in group algorithm." \
      " Nothing to analyze.\n",
      (int)self_->id.to_integer ());
  }

  return OK;
}

int
gams::algorithms::Formation_Sync::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::execute:" \
    " entering execute method\n");

  if (position_ >= 0)
  {
    int move = (int)barrier_.get_round ();

    if (move < (int)plan_.size ())
    {
      if (move < move_pivot_)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Formation_Sync::execute:" \
          " %d: Round %d: Moving along latitude to %s\n", position_,
          move, plan_[move].to_string ().c_str ());
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Formation_Sync::execute:" \
          " %d: Round %d: Moving along longitude to %s\n", position_,
          move, plan_[move].to_string ().c_str ());
      }

      platform_->move (plan_[move], platform_->get_accuracy ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Formation_Sync::execute:" \
      " device.%d does not have a position in group algorithm." \
      " Nothing to execute.\n",
      (int)self_->id.to_integer ());
  }
  return 0;
}

int
gams::algorithms::Formation_Sync::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Formation_Sync::plan:" \
    " entering plan method\n");

  return 0;
}
