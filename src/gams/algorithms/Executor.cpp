/**
 * Copyright (c) 2016 Carnegie Mellon University. All Rights Reserved.
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
 * @file Executor.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Implementation of the Executor algorithm. The Executor algorithm executes
 * other algorithms.
 **/

#include "gams/algorithms/Executor.h"

#include "gams/algorithms/AlgorithmFactoryRepository.h"
#include "gams/loggers/GlobalLogger.h"

#include <iostream>

gams::algorithms::BaseAlgorithm *
gams::algorithms::ExecutorFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::algorithms::ExecutorFactory::create:" \
     " creating Executor with %d args\n", args.size ());

  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::ExecutorFactory::create:" \
      " knowledge, sensors, platform, self, or agents are invalid\n");
  }

  return result;
}

gams::algorithms::Executor::Executor (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
  status_.init_vars (*knowledge, "executive", self->id.to_integer ());
  status_.init_variable_values ();
}

gams::algorithms::Executor::~Executor ()
{
}

void
gams::algorithms::Executor::operator= (Executor& rhs)
{
  if (this != &rhs)
  {
  }
}

int
gams::algorithms::Executor::analyze (void)
{
  int result (0);

  return result;
}
      
int
gams::algorithms::Executor::execute (void)
{
  int result (0);

  return result;
}

int
gams::algorithms::Executor::plan (void)
{
  int result (0);

  return result;
}
