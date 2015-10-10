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
 * @file FormationSync.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Definitions for FormationSync class
 * - The head agent runs some area coverage algorithm
 * - Followers perform formation flying around the head agent
 **/

#include "gams/algorithms/FormationSync.h"
#include "gams/algorithms/ControllerAlgorithmFactory.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "gams/algorithms/AlgorithmFactory.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;

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

gams::algorithms::BaseAlgorithm *
gams::algorithms::FormationSyncFactory::create (
const madara::knowledge::KnowledgeVector & args,
madara::knowledge::KnowledgeBase * knowledge,
platforms::BasePlatform * platform,
variables::Sensors * sensors,
variables::Self * self,
variables::Devices * devices)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSyncFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    utility::GPSPosition start;
    utility::GPSPosition end;
    std::vector <std::string> members;

    int formation_type = FormationSync::LINE;
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
          "gams::algorithms::FormationSyncFactory:" \
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
          "gams::algorithms::FormationSyncFactory:" \
          " setting end to %s\n", end.to_string ().c_str ());

        ++i;
      }
      else if (args[i] == "group" && i + 1 < args.size ())
      {
        group = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationSyncFactory:" \
          " setting group to %s\n", group.c_str ());

        std::string members_list_name = "group." + group + ".members";

        containers::StringVector member_list (members_list_name, *knowledge);

        member_list.copy_to (members);

        ++i;
      }
      else if (args[i] == "buffer" && i + 1 < args.size ())
      {
        buffer = args[i + 1].to_double ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationSyncFactory:" \
          " setting buffer to %f\n", buffer);

        ++i;
      }
      else if (args[i] == "barrier" && i + 1 < args.size ())
      {
        barrier = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationSyncFactory:" \
          " setting barrier to %s\n", barrier.c_str ());

        ++i;
      }
      else if (args[i] == "formation" && i + 1 < args.size ())
      {
        std::string formation_str = args[i + 1].to_string ();

        madara::utility::upper (formation_str);

        if (formation_str == "PYRAMID")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to PYRAMID\n");

          formation_type = FormationSync::PYRAMID;
        }
        else if (formation_str == "TRIANGLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to TRIANGLE\n");

          formation_type = FormationSync::TRIANGLE;
        }
        else if (formation_str == "RECTANGLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to RECTANGLE\n");

          formation_type = FormationSync::RECTANGLE;
        }
        else if (formation_str == "CIRCLE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to CIRCLE\n");

          formation_type = FormationSync::CIRCLE;
        }
        else if (formation_str == "LINE")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to LINE\n");

          formation_type = FormationSync::LINE;
        }
        else if (formation_str == "WING")
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
            " setting formation to WING\n");

          formation_type = FormationSync::WING;
        }
        else if (args[i + 1].is_integer_type ())
        {
          formation_type = (int)args[i + 1].to_integer ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationSyncFactory:" \
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
        "gams::algorithms::FormationSync::constructor:" \
        " No group specified. Using swarm.\n");

      Integer processes = (Integer)devices->size ();

      for (Integer i = 0; i < processes; ++i)
      {
        madara::knowledge::KnowledgeRecord temp ("device.");
        temp += i;
        members.push_back (temp.to_string ());
      }
    }

    result = new FormationSync (start, end, members, buffer,
      formation_type, barrier,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::FormationSync::FormationSync (
  utility::GPSPosition & start,
  utility::GPSPosition & end,
  const std::vector<std::string> & members,
  double buffer,
  int formation,
  std::string barrier_name,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self), start_ (start),
  end_ (end), members_ (members), buffer_ (buffer), formation_ (formation)
{
  status_.init_vars (*knowledge, "formation_sync", self->id.to_integer ());
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSync::constructor:" \
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
      "gams::algorithms::FormationSync::constructor:" \
      " device.%d does not have a position in group algorithm." \
      " Unable to participate in barrier.\n",
      (int)self_->id.to_integer ());

  }
}

void
gams::algorithms::FormationSync::generate_plan (int formation)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSync::constructor:" \
    " Generating plan\n");

  madara::knowledge::KnowledgeRecord temp ("device.");
  temp += self_->id.to_string ();

  position_ = this->get_position_in_member_list (temp.to_string (), members_);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MINOR,
    "gams::algorithms::FormationSync::constructor:" \
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
    "gams::algorithms::FormationSync::constructor:" \
    " Formation will move %.3f m lat, %.3f m long in %d moves\n",
    distances.x, distances.y, total_moves);

  // the type of movement will be based on sign of distance
  double latitude_move = distances.x < 0 ? -buffer_ : buffer_;
  double longitude_move = distances.y < 0 ? -buffer_ : buffer_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::FormationSync::constructor:" \
    " Will execute %.3f m in %d latitude moves and then" \
    " %.3f m in %d longitude moves\n",
    latitude_move, x_moves, longitude_move, y_moves);

  // a cartesian movement offset
  utility::Position movement;

  // the initial position for this specific device
  utility::GPSPosition init;
  utility::GPSPosition position_end;

  if (formation == TRIANGLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::constructor:" \
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
      "gams::algorithms::FormationSync::constructor:" \
      " Formation type is PYRAMID\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end = end_.to_gps_position (
      movement, end_);

  }
  else if (formation == RECTANGLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::constructor:" \
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
    position_end = end_.to_gps_position (
      movement, end_);

  }
  else if (formation == CIRCLE)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::constructor:" \
      " Formation type is CIRCLE\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end = end_.to_gps_position (
      movement, end_);

  }
  else if (formation == WING)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::constructor:" \
      " Formation type is WING\n");

    /**
    [ 0][  ][  ] if size % 2 == 1
    [  ][ 1][  ]   cols = size / 2 + 1
    [  ][  ][ 2]   col = position % cols
    [  ][ 3][  ]   row = position
    [ 4][  ][  ]

    else // even, 2 and 4 are outliers

    [ 0][  ][  ] if position != size - 1
    [  ][ 1][  ]   cols = size / 2
    [ 5][  ][ 2]   row = position
    [  ][ 3][  ]   col = position % cols
    [ 4][  ][  ] else
                   if (size == 4)
                     row = col = 2
                   else
                     row = size / 2
                     if size != 2
                       col = 0
                     else
                       col = 1
    **/


    int col, row;

    // if size is odd
    if (members_.size () % 2 == 1)
    {
      /**
       * size = 5, cols = 3
       * [0][ ][ ] pos = 0, row = 0, col = 0
       * [ ][1][ ] pos = 1, row = 1, col = 1
       * [ ][ ][2] pos = 2, row = 2, col = 2
       * [ ][3][ ] pos = 3, row = 3, col = 3 - 3 % 3 - 2 = 1
       * [4][ ][ ] pos = 4, row = 4, col = 3 - 4 % 3 - 2 = 3 - 1 - 2 = 0
       **/
      int cols = (int)members_.size () / 2 + 1;
      if (position_ >= cols)
      {
        col = cols - position_ % cols - 2;
      }
      else
      {
        col = position_ % cols;
      }
      row = position_;
    }
    // if size is even
    else
    {
      // handle everything before last position first
      if (position_ != members_.size () - 1)
      {
        /**
        * size = 6, cols = 3
        * [0][ ][ ] pos = 0, row = 0, col = 0
        * [ ][1][ ] pos = 1, row = 1, col = 1
        * [ ][ ][2] pos = 2, row = 2, col = 2
        * [ ][3][ ] pos = 3, row = 3, col = 3 - 3 % 3 - 2 = 1
        * [4][ ][ ] pos = 4, row = 4, col = 3 - 4 % 3 - 2 = 3 - 1 - 2 = 0
        **/
        int cols = (int)members_.size () / 2;
        row = position_;

        if (position_ >= cols)
        {
          col = cols - position_ % cols - 2;
        }
        else
        {
          col = position_ % cols;
        }
      }
      // handle the last position. 2 and 4 are outliers
      else
      {
        // In size == 4, we create a wedge rather than wing
        if (members_.size () == 4)
        {
          row = col = 2;
        }
        else
        {
          /**
          * size = 6, cols = 3
          * [0][ ][ ] 
          * [ ][1][ ] 
          * [5][ ][2] pos = 5, row = 2, col = 2
          * [ ][3][ ] 
          * [4][ ][ ] 
          **/

          // otherwise, we set the row to the middle of the formation
          row = (int)members_.size () / 2 - 1;

          // most formations will just use a drone at the far back and center
          if (members_.size () != 2)
          {
            col = 0;
          }
          // size == 2 will just increment the col 
          else
          {
            col = 1;
          }
        }
      }
    }

    // the initial position where the first two moves will be for this device
    movement.x = col * latitude_move * 2;
    movement.y = row * longitude_move * 2;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end = end_.to_gps_position (
      movement, end_);
  }
  // default is LINE
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::constructor:" \
      " Formation type is LINE\n");

    // the initial position where the first two moves will be for this device
    movement.x = position_ * latitude_move * 2;
    movement.y = 0;

    // the initial position for this specific device
    init = start_.to_gps_position (
      movement, start_);

    // the ending position for this specific device
    position_end = end_.to_gps_position (
      movement, end_);
  }

  utility::GPSPosition last (init);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::FormationSync::constructor:" \
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
    "gams::algorithms::FormationSync::constructor:" \
    " Generated the following plan:\n%s",
    full_plan_description.str ().c_str ());

}

gams::utility::GPSPosition
gams::algorithms::FormationSync::generate_position (utility::GPSPosition reference,
double angle, double distance)
{
  // compute the offset position
  utility::Position offset (
    distance * cos (angle), distance * sin (angle), 0);

  // return the new GPS position from the reference position
  return utility::GPSPosition::to_gps_position (
    offset, reference);
}

int
gams::algorithms::FormationSync::get_position_in_member_list (
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

gams::algorithms::FormationSync::~FormationSync ()
{
}

void
gams::algorithms::FormationSync::operator= (const FormationSync & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator= (rhs);

    start_ = rhs.start_;
    end_ = rhs.end_;
    members_ = rhs.members_;
    buffer_ = rhs.buffer_;
  }
}

int
gams::algorithms::FormationSync::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSync::analyze:" \
    " entering analyze method\n");

  if (position_ >= 0)
  {
    int round = barrier_.get_round ();

    barrier_.modify ();

    if (round < (int)plan_.size () && barrier_.is_done ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::FormationSync::analyze:" \
        " %d: Round %d of %d: Proceeding to next barrier round\n",
        position_, round, (int)plan_.size ());

      utility::GPSPosition current;
      current.from_container (self_->device.location);

      // for some reason, we have divergent functions for distance equality
      if (plan_[round].approximately_equal (current, platform_->get_accuracy ()))
      {
        barrier_.next ();
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::FormationSync::analyze:" \
        " %d: Round %d of %d: NOT proceeding to next barrier round\n",
        position_, round, (int)plan_.size ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::analyze:" \
      " device.%d does not have a position in group algorithm." \
      " Nothing to analyze.\n",
      (int)self_->id.to_integer ());
  }

  return OK;
}

int
gams::algorithms::FormationSync::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSync::execute:" \
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
          "gams::algorithms::FormationSync::execute:" \
          " %d: Round %d: Moving along latitude to %s\n", position_,
          move, plan_[move].to_string ().c_str ());
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationSync::execute:" \
          " %d: Round %d: Moving along longitude to %s\n", position_,
          move, plan_[move].to_string ().c_str ());
      }

      if (platform_->move (plan_[move], platform_->get_accuracy ()) ==
        gams::platforms::PLATFORM_ARRIVED &&
        move < (int)plan_.size () && barrier_.is_done ())
      {
        barrier_.next ();
      }
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::FormationSync::execute:" \
      " device.%d does not have a position in group algorithm." \
      " Nothing to execute.\n",
      (int)self_->id.to_integer ());
  }
  return 0;
}

int
gams::algorithms::FormationSync::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationSync::plan:" \
    " entering plan method\n");

  return 0;
}
