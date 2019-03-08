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
 * @file KarlEvaluator.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Implementation of the KarlEvaluator class. KarlEvaluator evaluates
 * a MADARA KaRL logic one or more times.
 **/

#include "gams/algorithms/KarlEvaluator.h"

#include "gams/loggers/GlobalLogger.h"

#include <iostream>

#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord  KnowledgeRecord;
typedef KnowledgeRecord::Integer            Integer;
typedef madara::knowledge::KnowledgeMap     KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::KarlEvaluatorFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::algorithms::KarlEvaluatorFactory::create:" \
    " creating KarlEvaluator with %d args\n", args.size ());

  BaseAlgorithm * result (0);
  
  std::string logic;
  std::string store_result;
  bool is_wait (false);
  double wait_time = -1;

  if (knowledge && sensors && platform && self)
  {
    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 'i':
        if (i->first == "is_wait" || i->first == "is_waiting")
        {
          is_wait = i->second.is_true ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::KarlEvaluatorFactory::create:" \
            " setting is_wait to %s\n", is_wait ? "true" : "false");
          break;
        }
        goto unknown;
      case 'l':
        if (i->first == "logic")
        {
          logic = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::KarlEvaluatorFactory::create:" \
            " setting logic to %s\n", logic.c_str ());
          break;
        }
        goto unknown;
      case 's':
        if (i->first == "store_result" || i->first == "store")
        {
          store_result = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::KarlEvaluatorFactory::create:" \
            " setting store_result to %s\n", store_result.c_str ());
          break;
        }
        goto unknown;
      case 'w':
        if (i->first == "wait" || i->first == "wait_time")
        {
          wait_time = i->second.to_double ();
          is_wait = true;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "gams::algorithms::KarlEvaluatorFactory::create:" \
            " setting wait_time to %f\n", wait_time);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::KarlEvaluator::create: " \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    if (logic != "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::KarlEvaluatorFactory::create:" \
        " args: logic=\"%s\", store_result=%s, is_wait=%s, wait_time=%f.\n",
        logic.c_str (), store_result.c_str (),
        is_wait ? "true" : "false", wait_time);

      result = new KarlEvaluator (logic, store_result, is_wait, wait_time,
        knowledge, platform, sensors, self, agents);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::KarlEvaluatorFactory::create:" \
        " logic is empty. No evaluation to perform.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::KarlEvaluatorFactory::create:" \
      " knowledge, sensors, platform, self, or agents are invalid\n");
  }

  return result;
}

gams::algorithms::KarlEvaluator::KarlEvaluator (
  const std::string & logic,
  const std::string & store_result,
  bool is_wait,
  double wait_time,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents),
  logic_ (logic), is_wait_ (is_wait), wait_time_ (wait_time),
  enforcer_ (wait_time, wait_time)
{
  if (knowledge)
  {
    status_.init_vars (*knowledge, "karl", self->agent.prefix);
    status_.init_variable_values ();

    // do not send modifications every eval. Let GAMS handle that.
    settings_.delay_sending_modifieds = true;

    // if store result is specified, add it to logic
    std::string actual_logic;
    if (store_result != "")
    {
      actual_logic = store_result + "=(" + logic + ")";
    }
    else
    {
      actual_logic = logic;
    }

#ifndef _MADARA_NO_KARL_
    // compile the logic for fast evaluation, which will be helpful if wait
    this->compiled_logic_ = knowledge->compile (actual_logic);
#endif


    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::KarlEvaluator: Create Success." \
      " args: logic=\"%s\", store_result=%s, is_wait=%s, wait_time=%f.\n",
      actual_logic.c_str (), store_result.c_str (),
      is_wait ? "true" : "false", wait_time);

  } // end knowledge
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::KarlEvaluator: FAILED TO CREATE. NULL KNOWLEDGE.\n");
  } // end !knowledge
}

gams::algorithms::KarlEvaluator::~KarlEvaluator ()
{
}

void
gams::algorithms::KarlEvaluator::operator= (KarlEvaluator& rhs)
{
  if (this != &rhs)
  {
#ifndef _MADARA_NO_KARL_
    compiled_logic_ = rhs.compiled_logic_;
#endif
    settings_ = rhs.settings_;
    logic_ = rhs.logic_;
    is_wait_ = rhs.is_wait_;
  }
}

int
gams::algorithms::KarlEvaluator::analyze (void)
{
  int result (0);

  return result;
}
      
int
gams::algorithms::KarlEvaluator::execute (void)
{
  int result (0);

  if (knowledge_ && status_.finished != 1)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "KarlEvaluator::execute: Evaluating logic.\n");

#ifndef _MADARA_NO_KARL_
    KnowledgeRecord eval_result = knowledge_->evaluate (
      compiled_logic_, settings_);
#else
    KnowledgeRecord eval_result;
#endif

    if (eval_result.is_false () && is_wait_ && wait_time_ > 0)
    {
      if (enforcer_.is_done ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "KarlEvaluator::execute: Wait time finished. Algorithm finished.\n");

        status_.finished = 1;
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "KarlEvaluator::execute: Evaluation returned false. "
          "Wait time not over. Looping\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "KarlEvaluator::execute: Evaluation finished. Algorithm finished.\n");

      status_.finished = 1;
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "KarlEvaluator::execute: Not evaluating. status.finished = %d\n",
      (int)*status_.finished);
  }

  return result;
}

int
gams::algorithms::KarlEvaluator::plan (void)
{
  int result (0);

  return result;
}
