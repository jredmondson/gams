/**
 * Copyright (c) 2014-2016 Carnegie Mellon University. All Rights Reserved.
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
#include "gams/loggers/GlobalLogger.h"
#include "Move.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

#include "madara/utility/Utility.h"

#include "gams/utility/ArgumentParser.h"
#include "gams/utility/Coordinate.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

typedef knowledge::KnowledgeRecord KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::MoveFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    std::vector <utility::Pose> poses;
    int repeat_times (0);
    double wait_time (0.0);

    KnowledgeMap::const_iterator poses_size_found =
      args.find ("locations.size");

    KnowledgeMap::const_iterator repeat_found = args.find ("repeat");

    KnowledgeMap::const_iterator wait_time_found = args.find ("wait_time");

    if (wait_time_found != args.end ())
    {
      wait_time = wait_time_found->second.to_double ();
    }
    else
    {
      wait_time_found = args.find ("wait");

      if (wait_time_found != args.end ())
      {
        wait_time = wait_time_found->second.to_double ();
      }
    }

    if (repeat_found != args.end ())
    {
      repeat_times = repeat_found->second.to_integer ();

      if (repeat_times < 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "MoveFactory::create" \
          " Setting repeat to forever\n");
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "MoveFactory::create" \
          " Setting repeat to %d times\n", repeat_times);
      }
    }

    if (poses_size_found != args.end ())
    {
      Integer num_locations = poses_size_found->second.to_integer ();
      if (num_locations > 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "MoveFactory::create" \
          " using locations.size of %d\n", (int)num_locations);

        poses.resize ((size_t)num_locations);

        /**
         * fastest way to iterate through this is to find the locations
         * iterator and then just put the position in the right place.
         * locations.0 is at the absolute lowest position possible in
         * the args map, see ASCII table for reason.
         **/
        KnowledgeMap::const_iterator next = args.find ("locations.0");

        /**
         * iterate over all locations (note we assume locations.size is
         * the only other legitimate variable with locations. prefix)
         **/
        for (; next != poses_size_found; ++next)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MINOR,
            "MoveFactory::create" \
            " processing arg %s = %s\n",
            next->first.c_str (), next->second.to_string ().c_str ());

          // We have already resized the array, so set the location
          KnowledgeRecord k_index (madara::utility::strip_prefix (
            next->first, "locations."));
          int index = (int)k_index.to_integer ();

          poses[index].frame (platform->get_frame ());

          poses[index].from_container (next->second.to_doubles ());

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MINOR,
            "MoveFactory::create" \
            " setting pose[%d] with [%s]\n",
            index, poses[index].to_string ().c_str (), next->first.c_str ());
        }

        if (poses.size () > 0)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "MoveFactory::create" \
            " creating Move algorithm with %d locations and %d repeats\n",
            (int)poses.size (), repeat_times);

          result = new Move (poses, repeat_times, wait_time,
            knowledge, platform, sensors, self, agents);
        }
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "MoveFactory::create" \
          " ERROR: locations.size must be a positive number\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "MoveFactory::create" \
        " ERROR: locations.size needs to be set to the number of waypoints\n");
    }
  }

  return result;
}

gams::algorithms::Move::Move (
  const std::vector <utility::Pose> & locations,
  int repeat,
  double wait_time,
  madara::knowledge::KnowledgeBase * knowledge, 
  platforms::BasePlatform * platform, variables::Sensors * sensors, 
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents), 
  poses_ (locations), repeat_ (repeat), move_index_ (0), cycles_ (0),
  wait_time_ (wait_time), waiting_ (false), finished_moving_ (false)
{
  status_.init_vars (*knowledge, "move", self->id.to_integer ());
  status_.init_variable_values ();
}

gams::algorithms::Move::~Move ()
{
}

void
gams::algorithms::Move::operator= (const Move & rhs)
{
  if (this != &rhs)
  {
    this->poses_ = rhs.poses_;
    this->repeat_ = rhs.repeat_;
    this->move_index_ = rhs.move_index_;
    this->cycles_ = rhs.cycles_;

    this->BaseAlgorithm::operator=(rhs);
  }
}

int
gams::algorithms::Move::analyze (void)
{
  int result (OK);

  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    if (self_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "algorithms::Move::analyze:" \
        " current pose is [%s, %s].\n",
        self_->agent.location.to_record ().to_string ().c_str (),
        self_->agent.orientation.to_record ().to_string ().c_str ());
    }

    if (status_.finished.is_false ())
    {
      if (move_index_ < poses_.size ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::analyze:" \
          " currently moving to pose %d -> [%s].\n",
          (int)move_index_,
          poses_[move_index_].to_string ().c_str ());

        // check our distance to the next location
        utility::Location loc = platform_->get_location ();
        utility::Location next_loc (platform_->get_frame (), poses_[move_index_]);

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "Move::analyze:" \
          " distance between points is %f (need %f accuracy)\n",
          loc.distance_to (next_loc), platform_->get_accuracy ());
      } // end next move is within the locations list
      else
      {
        if (!waiting_ || ACE_OS::gettimeofday () > end_time_)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_ERROR,
            "Move::analyze:" \
            " move_index_ is greater than possible locations.\n");

          result |= FINISHED;

          status_.finished.modify ();
        }

      } // end is not in the locations list
    } // end is not finished
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "Move::analyze:" \
        " Algorithm has finished previously. Rebroadcasting status.finished.\n");

      result |= FINISHED;

      status_.finished.modify ();
    } // end is finished
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "Move:analyze:" \
      " platform has not set movement_available to 1.\n");
  }

  return result;
}

int
gams::algorithms::Move::execute (void)
{
  int result (OK);

  int move_result (-1);

  if (waiting_ && ACE_OS::gettimeofday () > end_time_)
  {
    waiting_ = false;
  }
  
  if (platform_ && *platform_->get_platform_status ()->movement_available
      && !finished_moving_ && !waiting_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "Move::execute:" \
      " moving to pose [%s]\n",
      poses_[move_index_].to_string ().c_str ());

    // depending on type of pose that was set, pose, move or orient
    if (poses_[move_index_].is_set ())
    {
      if (!poses_[move_index_].is_location_set ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
          "Move::execute:" \
          " Only a orient is necessary to %s\n",
          utility::Orientation (poses_[move_index_]).to_string ().c_str ());

        move_result = platform_->orient (poses_[move_index_],
          platform_->get_accuracy ());

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::execute:" \
          " platform->orient returned %d.\n", move_result);
      }
      else if (!poses_[move_index_].is_orientation_set ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
          "Move::execute:" \
          " Only a location move is necessary to %s\n",
          utility::Location (poses_[move_index_]).to_string ().c_str ());

        move_result = platform_->move (poses_[move_index_],
          platform_->get_accuracy ());

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::execute:" \
          " platform->move returned %d.\n", move_result);
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
          "Move::execute:" \
          " A orientation and a location movement is necessary to %s\n",
          poses_[move_index_].to_string ().c_str ());

        move_result = platform_->pose (poses_[move_index_],
          platform_->get_accuracy ());

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::execute:" \
          " platform->pose returned %d.\n", move_result);

      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "Move::execute:" \
        " The platform was informed of new pose\n");

      if (move_result == platforms::PLATFORM_ARRIVED)
      {
        if (!waiting_ && wait_time_ > 0.0)
        {
          ACE_Time_Value delay;
          delay.set (wait_time_);
          end_time_ = ACE_OS::gettimeofday () + delay;
          waiting_ = true;
        }

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::execute:" \
          " platform operation was a success. Determining new moves.\n",
          move_result);

        // if we are at the end of the location list
        if (move_index_ == poses_.size () - 1)
        {
          ++cycles_;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "Move::execute:" \
            " finished waypoints cycle number %d of %d target cycles\n",
            cycles_, repeat_);

          if (cycles_ >= repeat_ && repeat_ >= 0)
          {
            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_MAJOR,
              "Move::execute:" \
              " algorithm is finished after %d cycles\n",
              cycles_);

            finished_moving_ = true;

            if (!waiting_)
            {
              result |= FINISHED;
              status_.finished = 1;
            }
          } // end if finished
          else
          {
            // reset the move index if we are supposed to cycle
            move_index_ = 0;

            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_MAJOR,
              "Move::execute:" \
              " proceeding to location 0 in next cycle.\n");

          } // end if not finished
        } // end if move_index_ == end of locations
        else
        {
          // go to the next location
          ++move_index_;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "Move::analyze:" \
            " proceeding to pose %d at [%s].\n",
            (int)move_index_,
            poses_[move_index_].to_string ().c_str ());

        } // if not the end of the list
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "Move::execute:" \
        " The pose is empty\n");

    }
  }
  else if (!waiting_)
  {

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "Move::execute:" \
      " No movements left. Not waiting. Finished.\n");

    result |= FINISHED;
    status_.finished = 1;
  }

  return result;
}

int
gams::algorithms::Move::plan (void)
{
  int result (OK);

  if (status_.finished.is_true ())
  {
    result |= FINISHED;
  }

  return result;
}
