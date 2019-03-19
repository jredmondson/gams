/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * @file Hold.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the Hold position class
 **/

#include "gams/algorithms/Hold.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
gams::algorithms::HoldFactory::create(
  const madara::knowledge::KnowledgeMap & /*args*/,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result(0);
  
  if (knowledge && sensors && platform && self)
  {
    result = new Hold(knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Hold::Hold(
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : BaseAlgorithm(knowledge, platform, sensors, self),
  pose_set_(false)
{
  status_.init_vars(*knowledge, "hold", self->agent.prefix);
  status_.init_variable_values();

  if (platform)
  {
    location_ = platform->get_location();
    orientation_ = platform->get_orientation();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Hold::constr:" \
      " setting hold location to [%s, %s]\n",
      location_.to_string().c_str(),
      orientation_.to_string().c_str());

    pose_set_ = true;
  }
}

gams::algorithms::Hold::~Hold()
{
}

void
gams::algorithms::Hold::operator=(const Hold & rhs)
{
  if (this != &rhs)
  {
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
  }
}


int
gams::algorithms::Hold::analyze(void)
{
  if (self_ && pose_set_)
  {
    pose::Position current(platform_->get_frame());
    current.from_container(self_->agent.location);

    double distance = current.distance_to(location_);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Hold::analyze:" \
      " distance from [%s] to hold position [%s] is %.2f\n",
      current.to_string().c_str(),
      location_.to_string().c_str(), distance);
  }
  else
  {
    if (platform_ && !pose_set_)
    {
      location_ = platform_->get_location();
      orientation_ = platform_->get_orientation();

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Hold::analyze:" \
        " setting hold location to [%s, %s]\n",
        location_.to_string().c_str(),
        orientation_.to_string().c_str());

      pose_set_ = true;
    }
  }

  return OK;
}
      

int
gams::algorithms::Hold::execute(void)
{
  int result = 0;

  if (platform_ && *platform_->get_platform_status()->movement_available)
  {
    std::vector <double> home = self_->agent.home.to_record().to_doubles();

    if (pose_set_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Hold::execute:" \
        " Hold location is %s. Moving to home.\n",
        location_.to_string().c_str());

      int move_result = platform_->move(
        location_, platform_->get_accuracy());

      executions_++;

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Hold::execute:" \
        " platform->move returned %d.\n", move_result);

      if (move_result == platforms::PLATFORM_ARRIVED)
      {
        status_.finished = 1;
        result |= FINISHED;
      }
    }
    else // if !pose_set
    {
      if (platform_)
      {
        location_ = platform_->get_location();
        orientation_ = platform_->get_orientation();

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::Hold::execute:" \
          " setting hold location to [%s, %s]\n",
          location_.to_string().c_str(),
          orientation_.to_string().c_str());

        pose_set_ = true;
      }
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Hold::execute:" \
      " platform is null. No movement possible.\n");
  }

  return result;
}


int
gams::algorithms::Hold::plan(void)
{
  return 0;
}
