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
 * @file Home.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the Home movement class
 **/

#include "gams/algorithms/Home.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
gams::algorithms::HomeFactory::create (
  const madara::knowledge::KnowledgeMap & /*args*/,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    result = new Home (knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Home::Home (
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : BaseAlgorithm (knowledge, platform, sensors, self)
{
  status_.init_vars (*knowledge, "takeoff", self->agent.prefix);
  status_.init_variable_values ();
}

gams::algorithms::Home::~Home ()
{
}

void
gams::algorithms::Home::operator= (const Home & rhs)
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
gams::algorithms::Home::analyze (void)
{
  if (self_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Home::analyze:" \
      " current pose is [%s, %s].\n",
      self_->agent.location.to_record ().to_string ().c_str (),
      self_->agent.orientation.to_record ().to_string ().c_str ());
  }

  return OK;
}
      

int
gams::algorithms::Home::execute (void)
{
  int result = 0;

  if (platform_)
  {
    std::vector <double> home = self_->agent.home.to_record ().to_doubles ();

    if (home.size () >= 2)
    {
      pose::Position destination (platform_->get_frame ());
      destination.from_container (self_->agent.home);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::Home::execute:" \
        " Home location is %s. Moving to home.\n",
        destination.to_string ().c_str ());

      int move_result = platform_->move (
        destination, platform_->get_accuracy ());

      executions_++;

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Home::execute:" \
        " platform->move returned %d.\n", move_result);

      if (move_result == platforms::PLATFORM_ARRIVED)
      {
        status_.finished = 1;
        result |= FINISHED;
      }
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Home::execute:" \
      " platform is null. No movement possible.\n");
  }

  return result;
}


int
gams::algorithms::Home::plan (void)
{
  return 0;
}
