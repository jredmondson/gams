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
#include "Wait.h"

#include <string>
#include <iostream>

using std::string;
using std::cerr;
using std::endl;

gams::algorithms::BaseAlgorithm *
gams::algorithms::WaitFactory::create (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    if (args.size () == 1 && 
      (args[0].is_integer_type () || args[0].is_double_type ()))
    {
      result = new Wait (args[0].to_double (), knowledge, platform, sensors, 
        self);
    }
    else
    {

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::WaitFactory::create:" \
        " invalid arguments\n");
    }
  }

  return result;
}

gams::algorithms::Wait::Wait (
  const double& length,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self),
  wait_time_ (length), end_time_ (ACE_OS::gettimeofday () + wait_time_)
{
  status_.init_vars (*knowledge, "wait", self->id.to_integer ());
  status_.init_variable_values ();
}

gams::algorithms::Wait::~Wait ()
{
}

void
gams::algorithms::Wait::operator= (const Wait & rhs)
{
  this->BaseAlgorithm::operator=(rhs);
}

int
gams::algorithms::Wait::analyze (void)
{
  const ACE_Time_Value now (ACE_OS::gettimeofday ());

  int ret_val (OK);
  if (now > end_time_)
  {
    ret_val = FINISHED;
    status_.finished = 1;
  }

  return ret_val;
}

int
gams::algorithms::Wait::execute (void)
{
  ++executions_;
  return 0;
}

int
gams::algorithms::Wait::plan (void)
{
  return 0;
}
