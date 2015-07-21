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

#include "Debug_Algorithm.h"
#include "gams/loggers/Global_Logger.h"

#include <iostream>

gams::algorithms::Base_Algorithm *
gams::algorithms::Debug_Algorithm_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * /*devices*/)
{
  std::string executions (".executions");

  if (args.size () != 0)
  {
    executions = args[0].to_string ();
  }

  return new Debug_Algorithm (knowledge, platform, sensors, self, executions);
}

gams::algorithms::Debug_Algorithm::Debug_Algorithm (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  const std::string & executions_location)
  : Base_Algorithm (knowledge, platform, sensors, self)
{
  if (knowledge)
  {
    status_.init_vars (*knowledge, "debug", self->id.to_integer ());
    k_executions_.set_name (executions_location, *knowledge);
  }
}

gams::algorithms::Debug_Algorithm::~Debug_Algorithm ()
{
}

void
gams::algorithms::Debug_Algorithm::operator= (const Debug_Algorithm & rhs)
{
  if (this != &rhs)
  {
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
    this->k_executions_ = rhs.k_executions_;
  }
}

int
gams::algorithms::Debug_Algorithm::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ": algorithm.analyze ()\n",
    *self_->id, *k_executions_);
  
  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.platform_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.platform_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (sensors_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.sensors_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.sensors_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (self_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.self_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.self_ is not set\n",
      *self_->id, *k_executions_);
  }

  status_.waiting = 1;
  status_.ok = 1;
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.ok == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.paused == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.waiting == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.deadlocked == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.failed == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.unknown == %" PRId64 "\n"
    ,
    *self_->id, *k_executions_, *status_.ok,
    *self_->id, *k_executions_, *status_.paused,
    *self_->id, *k_executions_, *status_.waiting,
    *self_->id, *k_executions_, *status_.deadlocked,
    *self_->id, *k_executions_, *status_.failed,
    *self_->id, *k_executions_, *status_.unknown);

  return OK;
}

int
gams::algorithms::Debug_Algorithm::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ": algorithm.execute ()\n",
    *self_->id, *k_executions_);
  
  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.platform_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.platform_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (sensors_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.sensors_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.sensors_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (self_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.self_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.self_ is not set\n",
      *self_->id, *k_executions_);
  }

  status_.waiting = 0;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.ok == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.paused == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.waiting == %" PRId64 "\n"
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.deadlocked == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.failed == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.unknown == %" PRId64 "\n",
    *self_->id, *k_executions_, *status_.ok,
    *self_->id, *k_executions_, *status_.paused,
    *self_->id, *k_executions_, *status_.waiting,
    *self_->id, *k_executions_, *status_.deadlocked,
    *self_->id, *k_executions_, *status_.failed,
    *self_->id, *k_executions_, *status_.unknown);

  if (platform_)
  {
    utility::GPS_Position next (1, 2, 3);
    platform_->move (next);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "%" PRId64 ":%" PRId64 ":  ERROR: platform_ is null. Cannot call move ().\n",
      *self_->id, *k_executions_);
  }

  ++k_executions_;

  return 0;
}


int
gams::algorithms::Debug_Algorithm::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ": algorithm.plan ()\n",
    *self_->id, *k_executions_);
  
  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.platform_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.platform_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (sensors_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.sensors_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.sensors_ is not set\n",
      *self_->id, *k_executions_);
  }

  if (self_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   algorithm.self_ is set\n",
      *self_->id, *k_executions_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ALWAYS,
      "%" PRId64 ":%" PRId64 ":   ERROR: algorithm.self_ is not set\n",
      *self_->id, *k_executions_);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.ok == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.paused == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.waiting == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.deadlocked == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.failed == %" PRId64 "\n",
    "%" PRId64 ":%" PRId64 ":     algorithm.status_.unknown == %" PRId64 "\n",
    *self_->id, *k_executions_, *status_.ok,
    *self_->id, *k_executions_, *status_.paused,
    *self_->id, *k_executions_, *status_.waiting,
    *self_->id, *k_executions_, *status_.deadlocked,
    *self_->id, *k_executions_, *status_.failed,
    *self_->id, *k_executions_, *status_.unknown);

  return 0;
}
