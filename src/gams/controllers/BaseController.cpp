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

#include "BaseController.h"

#include <iostream>
#include <sstream>

#include "ace/High_Res_Timer.h"
#include "ace/OS_NS_sys_time.h"
#include "madara/utility/Utility.h"
#include "gams/algorithms/AlgorithmFactoryRepository.h"
#include "gams/platforms/PlatformFactoryRepository.h"
#include "gams/loggers/GlobalLogger.h"
#include "gams/groups/GroupFactoryRepository.h"

// Java-specific header includes
#ifdef _GAMS_JAVA_
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams/utility/java/Acquire_VM.h"
#endif

using std::cerr;
using std::endl;

typedef  madara::knowledge::KnowledgeRecord::Integer  Integer;

gams::controllers::BaseController::BaseController (
  madara::knowledge::KnowledgeBase & knowledge,
  const ControllerSettings & settings)
  : algorithm_ (0), knowledge_ (knowledge), platform_ (0),
  settings_ (settings), checkpoint_count_ (0)
{
  init_vars (settings_.agent_prefix);

  // setup the platform and algorithm global repositories
  platforms::global_platform_factory->set_knowledge (&knowledge);
  platforms::global_platform_factory->set_platforms (&platforms_);
  platforms::global_platform_factory->set_self (&self_);

  algorithms::global_algorithm_factory->set_agents (&agents_);
  algorithms::global_algorithm_factory->set_knowledge (&knowledge_);
  algorithms::global_algorithm_factory->set_self (&self_);
  algorithms::global_algorithm_factory->set_sensors (&sensors_);
  algorithms::global_algorithm_factory->set_platform (platform_);

  // initialize the repository default mappings
  platforms::global_platform_factory->initialize_default_mappings ();
  algorithms::global_algorithm_factory->initialize_default_mappings ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::constructor:" \
    " default constructor called.\n");
}

gams::controllers::BaseController::~BaseController ()
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::destructor:" \
    " deleting algorithm.\n");
  delete algorithm_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::destructor:" \
    " deleting platform.\n");
  delete platform_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::destructor:" \
    " deleting accents.\n");
  for (algorithms::Algorithms::iterator i = accents_.begin ();
    i != accents_.end (); ++i)
  {
    delete *i;
  }
}

void gams::controllers::BaseController::add_platform_factory (
  const std::vector <std::string> & aliases,
  platforms::PlatformFactory * factory)
{
  platforms::global_platform_factory->add (aliases, factory);
}

void gams::controllers::BaseController::add_algorithm_factory (
  const std::vector <std::string> & aliases,
  algorithms::AlgorithmFactory * factory)
{
  algorithms::global_algorithm_factory->add (aliases, factory);
}

int
gams::controllers::BaseController::monitor (void)
{
  int result (0);

  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::monitor:" \
      " calling platform_->sense ()\n");

    result = platform_->sense ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::controllers::BaseController::monitor:" \
      " Platform undefined. Unable to call platform_->sense ()\n");
  }

  return result;
}

int
gams::controllers::BaseController::system_analyze (void)
{
  int return_value (0);
  //bool error (false);

  /**
   * Note that certain agent variables like command are kept local only.
   * @see gams::variables::Agent::init_vars
   * @see gams::variables::Swarm::init_vars
   **/

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::system_analyze:" \
    " checking agent and swarm commands\n");

  if (self_.agent.algorithm != "")
  {
    if (swarm_.algorithm_id != 0 &&
        self_.agent.last_algorithm_id == self_.agent.algorithm_id)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::system_analyze:" \
        " agent.algorithm already analyzed (last_algorithm=%d, cur_algorithm=%d)\n",
        (int)*self_.agent.last_algorithm_id,
        (int)*self_.agent.algorithm_id);
    }
    else
    {
      std::string prefix (self_.agent.algorithm_args.get_name () + ".");
      madara::knowledge::KnowledgeMap args (knowledge_.to_map_stripped (prefix));

      self_.agent.algorithm_args.sync_keys ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::system_analyze:" \
        " Processing agent command: %s\n", (*self_.agent.algorithm).c_str ());

      init_algorithm (self_.agent.algorithm.to_string (), args);

      self_.agent.last_algorithm = self_.agent.algorithm.to_string ();
      if (self_.agent.algorithm_id.is_true ())
      {
        self_.agent.last_algorithm_id = *self_.agent.algorithm_id;
      }

      // reset the command
      self_.agent.algorithm = "";
      self_.agent.algorithm_id = 0;
      self_.agent.last_algorithm_args.clear (true);
      self_.agent.algorithm_args.exchange (self_.agent.last_algorithm_args,
        true, true);
      self_.agent.algorithm_args.clear ();
    }
  }
  else if (swarm_.algorithm != "")
  {
    if (swarm_.algorithm_id != 0 && self_.agent.last_algorithm_id == swarm_.algorithm_id)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::system_analyze:" \
        " swarm.algorithm already analyzed (last_algorithm=%d, cur_algorithm=%d)\n",
        (int) *self_.agent.last_algorithm_id,
        (int)*swarm_.algorithm_id);
    }
    else
    {
      std::string prefix (swarm_.algorithm_args.get_name () + ".");
      madara::knowledge::KnowledgeMap args (knowledge_.to_map_stripped (prefix));

      swarm_.algorithm_args.sync_keys ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::system_analyze:" \
        " Processing swarm command: %s\n", (*swarm_.algorithm).c_str ());

      init_algorithm (swarm_.algorithm.to_string (), args);

      self_.agent.last_algorithm = swarm_.algorithm.to_string ();
      if (self_.agent.algorithm_id.is_true ())
      {
        self_.agent.last_algorithm_id = *swarm_.algorithm_id;
      }

      // reset the command
      swarm_.algorithm = "";
      self_.agent.last_algorithm_args.clear (true);
      self_.agent.algorithm_id = 0;
      swarm_.algorithm_args.exchange (self_.agent.last_algorithm_args,
        true, true);
      self_.agent.algorithm_args.clear ();
    }
  }

  if (self_.agent.madara_debug_level !=
    (Integer)madara::logger::global_logger->get_level ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Setting MADARA debug level to %d\n", (int)*self_.agent.madara_debug_level);

    madara::logger::global_logger->set_level ((int)*self_.agent.madara_debug_level);
  }

  if (self_.agent.gams_debug_level !=
    (Integer)gams::loggers::global_logger->get_level ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Setting GAMS debug level to %d\n", (int)*self_.agent.gams_debug_level);

    gams::loggers::global_logger->set_level ((int)*self_.agent.gams_debug_level);
  }

  return return_value;
}

int
gams::controllers::BaseController::analyze (void)
{
  int return_value (0);

  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::analyze:" \
      " calling platform_->analyze ()\n");

    return_value |= platform_->analyze ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::analyze:" \
      " Platform undefined. Unable to call platform_->analyze ()\n");
  }

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::analyze:" \
      " calling algorithm_->analyze ()\n");

    return_value |= algorithm_->analyze ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::analyze:" \
      " Algorithm undefined. Unable to call algorithm_->analyze ()\n");
  }


  if (accents_.size () > 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::analyze:" \
      " calling analyze on accents\n");
    for (algorithms::Algorithms::iterator i = accents_.begin ();
      i != accents_.end (); ++i)
    {
      (*i)->analyze ();
    }
  }

  return return_value;
}

int
gams::controllers::BaseController::plan (void)
{
  int return_value (0);

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::plan:" \
      " calling algorithm_->plan ()\n");

    return_value |= algorithm_->plan ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::plan:" \
      " Algorithm undefined. Unable to call algorithm_->plan ()\n");
  }

  if (accents_.size () > 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::plan:" \
      " calling plan on accents\n");

    for (algorithms::Algorithms::iterator i = accents_.begin ();
      i != accents_.end (); ++i)
    {
      (*i)->plan ();
    }
  }

  return return_value;
}

int
gams::controllers::BaseController::execute (void)
{
  int return_value (0);

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::execute:" \
      " calling algorithm_->execute ()\n");

    return_value |= algorithm_->execute ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::controllers::BaseController::execute:" \
      " Algorithm undefined. Unable to call algorithm_->execute ()\n");
  }

  if (accents_.size () > 0)
  {
    for (algorithms::Algorithms::iterator i = accents_.begin ();
      i != accents_.end (); ++i)
    {
      (*i)->execute ();
    }
  }

  return return_value;
}

int
gams::controllers::BaseController::run_once_ (void)
{
  // return value
  int return_value (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " calling monitor ()\n");

  // lock the context from any external updates
  madara::knowledge::ContextGuard guard (knowledge_);

  return_value |= monitor ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " after monitor (), %d modifications to send\n",
    (int)knowledge_.get_context ().get_modifieds ().size ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "%s\n",
    knowledge_.debug_modifieds ().c_str ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " calling analyze ()\n");

  return_value |= analyze ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " after analyze (), %d modifications to send\n",
    (int)knowledge_.get_context ().get_modifieds ().size ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "%s\n",
    knowledge_.debug_modifieds ().c_str ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " calling plan ()\n");

  return_value |= plan ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " after plan (), %d modifications to send\n",
    (int)knowledge_.get_context ().get_modifieds ().size ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "%s\n",
    knowledge_.debug_modifieds ().c_str ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " calling execute ()\n");

  return_value |= execute ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " after execute (), %d modifications to send\n",
    (int)knowledge_.get_context ().get_modifieds ().size ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::controllers::BaseController::run: modifieds=%s\n",
    knowledge_.debug_modifieds ().c_str ());

  return return_value;
}

int
gams::controllers::BaseController::run_once (void)
{
  // return value
  int return_value (run_once_ ());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " sending updates\n");

  // send modified values through network
  knowledge_.send_modifieds ();

  return return_value;
}


int
gams::controllers::BaseController::run (void)
{
  // check the debug levels and set accordingly
  if (settings_.madara_log_level >= 0)
  {
    self_.agent.madara_debug_level = settings_.madara_log_level;
    madara::logger::global_logger->set_level (settings_.madara_log_level);
  }
  if (settings_.gams_log_level >= 0)
  {
    self_.agent.gams_debug_level = settings_.gams_log_level;
    gams::loggers::global_logger->set_level (settings_.gams_log_level);
  }

  return run_hz (settings_.loop_hertz,
    settings_.run_time, settings_.send_hertz);
}


int
gams::controllers::BaseController::run (double loop_period,
  double max_runtime, double send_period)
{
  // return value
  int return_value (0);
  bool first_execute (true);
  const std::string checkpoint_prefix (
    settings_.checkpoint_prefix + "_" + settings_.agent_prefix + "_");

  // for checking for potential user commands
  double loop_hz = 1.0 / loop_period;
  double send_hz = 1.0 / send_period;

  self_.agent.loop_hz = loop_hz;
  self_.agent.send_hz = send_hz;

  // if user specified non-positive, then we are to use loop_period
  if (send_period <= 0)
  {
    send_period = loop_period;
  }

  // loop every period until a max run time has been reached
  ACE_Time_Value current = ACE_OS::gettimeofday ();
  ACE_Time_Value max_wait, sleep_time, next_epoch;
  ACE_Time_Value send_sleep_time, send_next_epoch;
  ACE_Time_Value poll_frequency, send_poll_frequency;
  ACE_Time_Value last (current), last_send (current);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " loop_period: %fs, max_runtime: %fs, send_period: %fs\n",
    loop_period, max_runtime, send_period);

  if (settings_.checkpoint_strategy != CHECKPOINT_NONE)
  {
    std::stringstream filename;
    filename << checkpoint_prefix << checkpoint_count_ << ".kb";

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::run:" \
      " saving initial context to %s%d.kb\n",
      checkpoint_prefix.c_str (), checkpoint_count_);

    knowledge_.save_context (filename.str ());
    ++checkpoint_count_;
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " calling system_analyze ()\n");
  return_value |= system_analyze ();

  if (loop_period >= 0.0)
  {
    max_wait.set (max_runtime);
    max_wait = current + max_wait;

    poll_frequency.set (loop_period);
    send_poll_frequency.set (send_period);
    next_epoch = current + poll_frequency;
    send_next_epoch = current;

    //unsigned int iterations = 0;
    while (first_execute || max_runtime < 0 || current < max_wait)
    {
      // return value should be last return value of mape loop
      return_value = run_once_ ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::run:" \
        " calling system_analyze ()\n");
      return_value |= system_analyze ();

      if (settings_.checkpoint_strategy == CHECKPOINT_EVERY_LOOP)
      {
        std::stringstream filename;
        filename << checkpoint_prefix << checkpoint_count_ << ".kb";

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " saving context after loop to %s%d.kb\n",
          checkpoint_prefix.c_str (), checkpoint_count_);

        knowledge_.save_context (filename.str ());
        ++checkpoint_count_;
      }

      // grab current time
      current = ACE_OS::gettimeofday ();

      // run will always try to send at least once
      if (first_execute || current > send_next_epoch)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " sending updates\n");

        if (settings_.checkpoint_strategy == CHECKPOINT_EVERY_SEND)
        {
          std::stringstream filename;
          filename << checkpoint_prefix << checkpoint_count_ << ".kb";

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "gams::controllers::BaseController::run:" \
            " saving context before send_modifieds to %s%d.kb\n",
            checkpoint_prefix.c_str (), checkpoint_count_);

          knowledge_.save_context (filename.str ());
          ++checkpoint_count_;
        }

        // send modified values through network
        knowledge_.send_modifieds ();

        // setup the next send epoch
        if (send_period > 0)
        {
          while (send_next_epoch < current)
            send_next_epoch += send_poll_frequency;
        }
      }

      // check to see if we need to sleep for next loop epoch
      if (loop_period > 0.0 && current < next_epoch)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
          "gams::controllers::BaseController::run:" \
          " sleeping until next epoch\n");

        madara::utility::sleep (next_epoch - current);
      }

      // setup the next 
      next_epoch += poll_frequency;

      // run will always execute at least one time. Update flag for execution.
      if (first_execute)
        first_execute = false;

      // if send herz difference is more than .001 hz different, change epoch
      if (!madara::utility::approx_equal (
        send_hz, self_.agent.send_hz.to_double (), 0.001))
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " Changing send hertz from %.2f to %.2f\n", send_hz,
          self_.agent.send_hz.to_double ());

        send_hz = self_.agent.send_hz.to_double ();
        send_period = 1 / send_hz;
        send_poll_frequency.set (send_period);
        send_next_epoch = current + send_poll_frequency;
      }

      // if loop herz difference is more than .001 hz different, change epoch
      if (!madara::utility::approx_equal (
        loop_hz, self_.agent.loop_hz.to_double (), 0.001))
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " Changing loop hertz from %.2f to %.2f\n", loop_hz,
          self_.agent.loop_hz.to_double ());

        loop_hz = self_.agent.loop_hz.to_double ();
        loop_period = 1 / loop_hz;
        poll_frequency.set (loop_period);
        next_epoch = current + poll_frequency;
      }

      // if our loop hertz is not fast enough for sending, change it
      if (send_hz > loop_hz)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " Changing loop hertz from %.2f to %.2f\n", loop_hz,
          send_hz);

        loop_hz = send_hz;
        loop_period = 1 / loop_hz;
        poll_frequency.set (loop_period);
        next_epoch = current + poll_frequency;

        // update container so others know we are changing rate
        self_.agent.loop_hz = loop_hz;
      }
    }
  }

  return return_value;
}

void
gams::controllers::BaseController::init_accent (const std::string & algorithm,
const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_accent:" \
    " initializing accent %s\n", algorithm.c_str ());

  if (algorithm == "")
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::controllers::BaseController::init_accent:" \
      " ERROR: accent name is null\n");
  }
  else
  {
    // create new accent pointer and algorithm factory
    algorithms::BaseAlgorithm * new_accent (0);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_accent:" \
      " factory is creating accent %s\n", algorithm.c_str ());

    algorithms::global_algorithm_factory->set_agents (&agents_);
    algorithms::global_algorithm_factory->set_knowledge (&knowledge_);
    algorithms::global_algorithm_factory->set_self (&self_);
    algorithms::global_algorithm_factory->set_sensors (&sensors_);
    algorithms::global_algorithm_factory->set_platform (platform_);

    new_accent = algorithms::global_algorithm_factory->create (algorithm, args);

    if (new_accent)
    {
      accents_.push_back (new_accent);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::controllers::BaseController::init_accent:" \
        " ERROR: created accent is null.\n");
    }
  }
}

void gams::controllers::BaseController::clear_accents (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::clear_accents:" \
    " deleting and clearing all accents\n");

  for (unsigned int i = 0; i < accents_.size (); ++i)
  {
    delete accents_[i];
  }

  accents_.clear ();
}
void
gams::controllers::BaseController::init_algorithm (
const std::string & algorithm, const madara::knowledge::KnowledgeMap & args)
{
  // initialize the algorithm

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_algorithm:" \
    " initializing algorithm %s\n", algorithm.c_str ());

  if (algorithm == "")
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "Algorithm is empty.\n\n" \
      "SUPPORTED ALGORITHMS:\n" \
      "  bridge | bridging\n" \
      "  random area coverage\n" \
      "  priotized area coverage\n"
      );
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm:" \
      " deleting old algorithm\n");

    delete algorithm_;

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm:" \
      " factory is creating algorithm %s\n", algorithm.c_str ());

    algorithms::global_algorithm_factory->set_agents (&agents_);
    algorithms::global_algorithm_factory->set_knowledge (&knowledge_);
    algorithms::global_algorithm_factory->set_self (&self_);
    algorithms::global_algorithm_factory->set_sensors (&sensors_);
    algorithms::global_algorithm_factory->set_platform (platform_);

    algorithm_ = algorithms::global_algorithm_factory->create (algorithm, args);

    if (algorithm_ == 0)
    {
      // the user is going to expect this kind of error to be printed immediately
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm:" \
        " failed to create algorithm\n");
    }
    else
    {
#ifdef _GAMS_JAVA_
      algorithms::JavaAlgorithm * jalg =
        dynamic_cast <algorithms::JavaAlgorithm *> (algorithm_);

      if (jalg)
      {
        // Acquire the Java virtual machine
        gams::utility::java::Acquire_VM jvm;

        jclass controller_class = gams::utility::java::find_class (
          jvm.env, "com/gams/controllers/BaseController");
        jobject alg = jalg->get_java_instance ();
        jclass alg_class = jvm.env->GetObjectClass (alg);
        
        jmethodID init_call = jvm.env->GetMethodID (alg_class,
          "init", "(Lcom/gams/controllers/BaseController;)V");
        jmethodID controllerFromPointerCall = jvm.env->GetStaticMethodID (
          controller_class,
          "fromPointer", "(JZ)Lcom/gams/controllers/BaseController;");

        if (init_call && controllerFromPointerCall)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "gams::controllers::BaseController::init_algorithm:" \
            " Calling BaseAlgorithm init method.\n");
          jobject controller = jvm.env->CallStaticObjectMethod (controller_class,
            controllerFromPointerCall, (jlong)this, (jboolean)false); 

          jvm.env->CallVoidMethod (
            alg, init_call, controller);

          jvm.env->DeleteLocalRef (controller);
        }
        else
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_ERROR,
            "gams::controllers::BaseController::init_algorithm:" \
            " ERROR. Could not locate init and fromPointer calls in "
            "BaseController. Unable to initialize algorithm.\n");
        }

        jvm.env->DeleteLocalRef (alg_class);
        jvm.env->DeleteLocalRef (alg);
        jvm.env->DeleteWeakGlobalRef (controller_class);
      }
      else
      {
        init_vars (*algorithm_);
      }
#else
      init_vars (*algorithm_);
#endif
    }
  }
}

void
gams::controllers::BaseController::init_platform (
  const std::string & platform,
  const madara::knowledge::KnowledgeMap & args)
{
  // initialize the platform

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform:" \
    " initializing platform %s\n", platform.c_str ());

  if (platform == "")
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "Platform is empty.\n\n" \
      "SUPPORTED PLATFORMS:\n" \
      "  drone-rk\n" \
      "  vrep\n" \
      );
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " deleting old platform\n");

    delete platform_;
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " factory is creating platform %s\n", platform.c_str ());

    platforms::global_platform_factory->set_knowledge (&knowledge_);
    platforms::global_platform_factory->set_platforms (&platforms_);
    platforms::global_platform_factory->set_self (&self_);
    platforms::global_platform_factory->set_sensors (&sensors_);

    platform_ = platforms::global_platform_factory->create (platform, args);

    if (platform_)
    {
      init_vars (*platform_);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_platform:" \
        " platform creation failed.\n");
    }

    if (algorithm_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_platform:" \
        " algorithm is initialized. Updating to platform\n");

      algorithm_->set_platform (platform_);
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " Updating algorithm factory's platform\n");

    algorithms::global_algorithm_factory->set_platform (platform_);
  }
}

void gams::controllers::BaseController::init_algorithm (algorithms::BaseAlgorithm * algorithm)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_algorithm:" \
    " deleting old algorithm\n");

  delete algorithm_;
  algorithm_ = algorithm;

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm:" \
      " initializing vars in algorithm\n");

    init_vars (*algorithm_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm:" \
      " algorithm was reset to none\n");
  }
}


void gams::controllers::BaseController::init_platform (platforms::BasePlatform * platform)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform:" \
    " deleting old platform\n");

  delete platform_;
  platform_ = platform;

  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " initializing vars in platform\n");

    init_vars (*platform_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " platform was reset to none\n");
  }

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " algorithm is already initialized. Updating to new platform\n");

    algorithm_->set_platform (platform_);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform:" \
    " Updating algorithm factory's platform\n");

  algorithms::global_algorithm_factory->set_platform (platform_);
}


void
gams::controllers::BaseController::configure (
  const ControllerSettings & settings)
{
  settings_ = settings;

  if (settings_.madara_log_level >= 0)
  {
    self_.agent.madara_debug_level = settings_.madara_log_level;
    madara::logger::global_logger->set_level (settings_.madara_log_level);
  }
  if (settings_.gams_log_level >= 0)
  {
    self_.agent.gams_debug_level = settings_.gams_log_level;
    gams::loggers::global_logger->set_level (settings_.gams_log_level);
  }
}

#ifdef _GAMS_JAVA_

void gams::controllers::BaseController::init_algorithm (jobject algorithm)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_algorithm (java):" \
    " deleting old algorithm\n");

  delete algorithm_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_algorithm (java):" \
    " creating new Java algorithm\n");

  algorithm_ = new gams::algorithms::JavaAlgorithm (algorithm);

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm (java):" \
      " initializing vars for algorithm\n");

    init_vars (*algorithm_);
  }
}


void gams::controllers::BaseController::init_platform (jobject platform)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform (java):" \
    " deleting old platform\n");

  delete platform_;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform (java):" \
    " creating new Java platform\n");

  platform_ = new gams::platforms::JavaPlatform (platform);

  if (platform_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform (java):" \
      " initializing vars for platform\n");

    init_vars (*platform_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform (java):" \
      " platform creation failed.\n");
  }

  if (algorithm_)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform (java):" \
      " algorithm is initialized. Updating to platform\n");

    algorithm_->set_platform (platform_);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_platform (java):" \
    " Updating algorithm factory's platform\n");

  algorithms::global_algorithm_factory->set_platform (platform_);
}

#endif

void
gams::controllers::BaseController::init_vars (
  const std::string & self_prefix,
  const std::string & group_name)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " %s self, %s group\n",
    self_prefix.c_str (), group_name.c_str ());

  groups::GroupFactoryRepository factory (&knowledge_);
  groups::GroupBase * group = factory.create (group_name);

  init_vars (self_prefix, group);

  delete group;
}

void
gams::controllers::BaseController::init_vars (
  const std::string & self_prefix,
  const groups::GroupBase * group)
{
  if (group)
  {
    madara_logger_ptr_log (
      gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_vars:" \
      " %s self, %s group\n",
      self_prefix.c_str (), group->get_prefix ().c_str ());

    variables::init_vars (agents_, knowledge_, *group);
  }
  else
  {
    madara_logger_ptr_log (
      gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_vars:" \
      " %s self, no group\n",
      self_prefix.c_str ());
  }

  self_.init_vars (knowledge_, self_prefix);
  swarm_.init_vars (knowledge_);

  if (settings_.madara_log_level >= 0)
  {
    self_.agent.madara_debug_level = settings_.madara_log_level;
    madara::logger::global_logger->set_level (settings_.madara_log_level);
  }
  if (settings_.gams_log_level >= 0)
  {
    self_.agent.gams_debug_level = settings_.gams_log_level;
    gams::loggers::global_logger->set_level (settings_.gams_log_level);
  }
}

void
gams::controllers::BaseController::init_vars (
const Integer & id,
const Integer & processes)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " %" PRId64 " id, %" PRId64 " processes\n", id, processes);

  // initialize the agents, swarm, and self variables
  variables::init_vars (agents_, knowledge_, processes);
  swarm_.init_vars (knowledge_, processes);
  self_.init_vars (knowledge_, id);

  if (settings_.madara_log_level >= 0)
  {
    self_.agent.madara_debug_level = settings_.madara_log_level;
    madara::logger::global_logger->set_level (settings_.madara_log_level);
  }
  if (settings_.gams_log_level >= 0)
  {
    self_.agent.gams_debug_level = settings_.gams_log_level;
    gams::loggers::global_logger->set_level (settings_.gams_log_level);
  }
}

void
gams::controllers::BaseController::init_vars (platforms::BasePlatform & platform)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " initializing platform's vars\n");

  platform.knowledge_ = &knowledge_;
  platform.self_ = &self_;
  platform.sensors_ = &sensors_;

  algorithms::global_algorithm_factory->set_platform (&platform);
}


void
gams::controllers::BaseController::init_vars (algorithms::BaseAlgorithm & algorithm)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " initializing algorithm's vars\n");

  algorithm.agents_ = &agents_;
  algorithm.knowledge_ = &knowledge_;
  algorithm.platform_ = platform_;
  algorithm.self_ = &self_;
  algorithm.sensors_ = &sensors_;
}

gams::algorithms::BaseAlgorithm *
gams::controllers::BaseController::get_algorithm (void)
{
  return algorithm_;
}

gams::platforms::BasePlatform *
gams::controllers::BaseController::get_platform (void)
{
  return platform_;
}
