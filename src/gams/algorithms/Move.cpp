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
#include "gams/loggers/GlobalLogger.h"
#include "Move.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

#include "gams/utility/ArgumentParser.h"

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
    std::vector <utility::Position> locations;
    int repeat_times (0);

    KnowledgeMap::const_iterator locations_size_found =
      args.find ("locations.size");

    KnowledgeMap::const_iterator repeat_found = args.find ("repeat");

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

    if (locations_size_found != args.end ())
    {
      Integer num_locations = locations_size_found->second.to_integer ();
      if (num_locations > 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "MoveFactory::create" \
          " using locations.size of %d\n", (int)num_locations);

        locations.resize ((size_t)num_locations);

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
        for (; next != locations_size_found; ++next)
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

          locations[index] = utility::Position::from_record (next->second);

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MINOR,
            "MoveFactory::create" \
            " setting location[%d] with [%s]\n",
            index, locations[index].to_string ().c_str (), next->first.c_str ());
        }

        if (locations.size () > 0)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "MoveFactory::create" \
            " creating Move algorithm with %d locations and %d repeats\n",
            (int)locations.size (), repeat_times);

          result = new Move (locations, repeat_times,
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
  const std::vector <utility::Position> & locations,
  int repeat,
  madara::knowledge::KnowledgeBase * knowledge, 
  platforms::BasePlatform * platform, variables::Sensors * sensors, 
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents), 
  locations_ (locations), repeat_ (repeat), move_index_ (0)
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
    this->locations_ = rhs.locations_;
    this->repeat_ = rhs.repeat_;
    this->move_index_ = rhs.move_index_;

    this->BaseAlgorithm::operator=(rhs);
  }
}

int
gams::algorithms::Move::analyze (void)
{
  int result (OK);

  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    if (status_.finished.is_false ())
    {
      if (move_index_ < locations_.size ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Move::analyze:" \
          " currently moving to location %d -> [%s].\n",
          (int)move_index_,
          locations_[move_index_].to_string ().c_str ());

        // check our distance to the next location
        utility::Location loc = platform_->get_location ();
        utility::Location next_loc (platform_->get_frame (),
          locations_[move_index_].y, locations_[move_index_].x,
          locations_[move_index_].z);

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "Move::analyze:" \
          " distance between points is %f (need %f accuracy)\n",
          loc.distance_to (next_loc), platform_->get_accuracy ());

        if (loc.approximately_equal (next_loc, platform_->get_accuracy ()))
        {
          // if we are at the end of the location list
          if (move_index_ == locations_.size () - 1)
          {
            ++cycles_;

            madara_logger_ptr_log (gams::loggers::global_logger.get (),
              gams::loggers::LOG_MAJOR,
              "Move::analyze:" \
              " finished waypoints cycle number %d of %d target cycles\n",
              cycles_, repeat_);

            if (cycles_ >= repeat_ && repeat_ >= 0)
            {
              madara_logger_ptr_log (gams::loggers::global_logger.get (),
                gams::loggers::LOG_MAJOR,
                "Move::analyze:" \
                " algorithm is finished after %d cycles\n",
                cycles_);

              result |= FINISHED;
              status_.finished = 1;
            } // end if finished
            else
            {
              // reset the move index if we are supposed to cycle
              move_index_ = 0;

              madara_logger_ptr_log (gams::loggers::global_logger.get (),
                gams::loggers::LOG_MAJOR,
                "Move::analyze:" \
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
              " proceeding to location %d at [%s].\n",
              (int)move_index_,
              locations_[move_index_].to_string ().c_str ());

          } // if not the end of the list
        } // end if location is approximately equal to next location
      } // end next move is within the locations list
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "Move::analyze:" \
          " ERROR: move_index_ is greater than possible locations.\n");

        result |= FINISHED;

        status_.finished.modify ();
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

  bool is_finished = status_.finished == 1;

  if (platform_ && *platform_->get_platform_status ()->movement_available
    && !is_finished)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "Move::execute:" \
      " calling platform->move(\"%s\")\n",
      locations_[move_index_].to_string ().c_str ());

    // allow GPSPosition to do the conversion for lat/lon
    utility::GPSPosition next = locations_[move_index_];

    platform_->move (next);
  }
  else if (is_finished)
  {
    result |= OK;
  }

  return result;
}

int
gams::algorithms::Move::plan (void)
{
  int result (OK);

  if (status_.finished.is_false ())
  {
    result |= FINISHED;
  }

  return result;
}
