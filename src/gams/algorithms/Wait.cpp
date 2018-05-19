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

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer            Integer;
typedef madara::knowledge::KnowledgeMap     KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::WaitFactory::create (
const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);
  double wait_time = 0.0;
  
  if (knowledge && sensors && platform && self)
  {

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 't':
        if (i->first == "time")
        {
          wait_time = i->second.to_double ();
        }
      }
    }

    result = new Wait (wait_time, knowledge, platform, sensors, self);
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
  enforcer_ (length, length)
{
  status_.init_vars (*knowledge, "wait", self->agent.prefix);
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "algorithms::Wait::constr:" \
    " waiting for %.2f seconds.\n", length);
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
  int ret_val (OK);
  if (enforcer_.is_done ())
  {
    ret_val = FINISHED;
    status_.finished = 1;

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::Wait::analyze:" \
      " finished set to 1. Algorithm ready to move on.\n");
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "algorithms::Wait::analyze:" \
      " we have not waited long enough.\n");
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
