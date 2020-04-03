/**
 * Copyright(c) 2020 Galois. All Rights Reserved.
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
 * 3. The names "Galois," "Carnegie Mellon University," "SEI" and/or "Software
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

#include "HiveController.h"

#include <iostream>
#include <sstream>

#include "madara/utility/Utility.h"
#include "gams/algorithms/AlgorithmFactoryRepository.h"
#include "gams/platforms/PlatformFactoryRepository.h"
#include "gams/loggers/GlobalLogger.h"
#include "madara/utility/EpochEnforcer.h"

// Java-specific header includes
#ifdef _GAMS_JAVA_
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams/utility/java/Acquire_VM.h"
#endif

namespace knowledge = madara::knowledge;
namespace transport = madara::transport;

typedef  madara::knowledge::KnowledgeRecord::Integer  Integer;

typedef  madara::utility::EpochEnforcer<std::chrono::steady_clock> EpochEnforcer;

gams::controllers::HiveController::HiveController()
  : offset_(0), hive_(0), settings_()
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::constructor:" \
    " default constructor.\n");
}

gams::controllers::HiveController::HiveController(
   madara::knowledge::Hive & hive,
   size_t init_id, size_t num_controllers, const ControllerSettings & settings)
  : offset_(init_id), hive_(&hive), settings_(settings)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::constructor:" \
    " creating %d controllers.\n");

  resize(init_id, num_controllers);
}

gams::controllers::HiveController::~HiveController()
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::destructor:" \
    " deleting controllers.\n");

  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    delete controllers_[i];
  }
}

void gams::controllers::HiveController::add_platform_factory(
  const std::vector <std::string> & aliases,
  platforms::PlatformFactory * factory)
{
  gams::platforms::global_platform_factory()->add(aliases, factory);
}

void gams::controllers::HiveController::add_algorithm_factory(
  const std::vector <std::string> & aliases,
  algorithms::AlgorithmFactory * factory)
{
  gams::algorithms::global_algorithm_factory()->add(aliases, factory);
}

void
gams::controllers::HiveController::add_transports(
  const madara::transport::QoSTransportSettings & source_settings)
{
  if (hive_ != 0)
  {
    // if no transport, short circuit
    if (source_settings.type == madara::transport::NO_TRANSPORT)
    {
      return;
    }

    madara::transport::QoSTransportSettings settings(source_settings);

    std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

    // if using a unicast protocol and we have 0 or 1 host, extrapolate
    if ((settings.type == madara::transport::UDP ||
      settings.type == madara::transport::ZMQ) &&
      settings.hosts.size() <= 1)
    {
      std::string key;
      std::string host = "127.0.0.1";
      std::string port_string = "30000";
      unsigned short port_short = 30000;

      if (settings.hosts.size() == 1)
      {
        madara::utility::split_hostport_identifier(
          settings.hosts[0], host, port_string);
        std::stringstream buffer(port_string);
        buffer >> port_short;
      }

      settings.hosts.resize(kbs.size());

      for (unsigned short i = 0; i < kbs.size(); ++i)
      {
        madara::utility::merge_hostport_identifier(
          settings.hosts[i], host, port_short + i);
      } // for i < number of kbs
    } // if settings are reasonable

    // need to update this for unicast host:port splitting
    for (size_t i = 0; i < controllers_.size(); ++i)
    {
      kbs[i + offset_].attach_transport(
        kbs[i + offset_].get(".prefix").to_string(), settings);
    } // for i < kbs.size()
  } // if hive is valid
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::add_transports:" \
      " no hive has been set. Operation is a no-op.\n");
  }
}

void gams::controllers::HiveController::clear_knowledge(void)
{
  if (hive_ != 0)
  {
    std::vector<knowledge::KnowledgeBase> kbs = hive_->get_kbs();
    for (size_t i = 0; i < controllers_.size(); ++i)
    {
      kbs[i + offset_].clear();
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::clear_knowledge:" \
      " no hive has been set. Operation is a no-op.\n");
  }
}

void
gams::controllers::HiveController::clear_accents(size_t controller_index)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::clear_accents:" \
    " deleting and clearing all accents in %d controller\n",
    (int)controller_index);

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->clear_accents();
  }
}

void
gams::controllers::HiveController::evaluate(const std::string & logic,
  const madara::knowledge::EvalSettings & settings)
{
#ifndef _MADARA_NO_KARL_
  if (hive_ != 0)
  {
    std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::HiveController::evaluate:" \
      " evaluating logic in all controllers\n");

    for (size_t i = 0; i < controllers_.size(); ++i)
    {
      kbs[i + offset_].evaluate(logic, settings);
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::evaluate:" \
      " no hive has been set. Operation is a no-op.\n");
  }
#endif
}

void
gams::controllers::HiveController::evaluate(size_t controller_index,
  const std::string & logic,
  const madara::knowledge::EvalSettings & settings)
{
#ifndef _MADARA_NO_KARL_
  
  if (hive_ != 0)
  {
    std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::HiveController::evaluate:" \
      " evaluating logic in controller %d\n", (int)controller_index);

    if (controller_index < controllers_.size())
    {
      kbs[controller_index + offset_].evaluate(logic, settings);
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::evaluate:" \
      " no hive has been set. Operation is a no-op.\n");
  }
#endif
}

void
gams::controllers::HiveController::init_accent(
  const std::string & algorithm,
  const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_accent:" \
    " initializing all controllers with accent %s\n", algorithm.c_str());

  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    controllers_[i]->init_accent(algorithm, args);
  }
}

void
gams::controllers::HiveController::init_accent(size_t controller_index,
  const std::string & algorithm,
  const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_accent:" \
    " initializing accent %d:%s\n", (int)controller_index, algorithm.c_str());

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_accent(algorithm, args);
  }
}

void
gams::controllers::HiveController::init_algorithm(
  const std::string & algorithm,
  const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_algorithm:" \
    " initializing all controllers with algorithm %s\n", algorithm.c_str());

  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    controllers_[i]->init_algorithm(algorithm, args);
  }
}

void
gams::controllers::HiveController::init_algorithm(
  size_t controller_index, const std::string & algorithm,
  const madara::knowledge::KnowledgeMap & args)
{
  // initialize the algorithm

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controll ers::HiveController::init_algorithm:" \
    " initializing algorithm %d:%s\n",
    (int)controller_index, algorithm.c_str());

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_algorithm(algorithm, args);
  }
}

void
gams::controllers::HiveController::init_platform(
  const std::string & platform,
  const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_platform:" \
    " initializing all controllers with platform %s\n", platform.c_str());

  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    controllers_[i]->init_platform(platform, args);
  }
}

void
gams::controllers::HiveController::init_platform(
  size_t controller_index, const std::string & platform,
  const madara::knowledge::KnowledgeMap & args)
{
  // initialize the platform

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_platform:" \
    " initializing platform %d:%s\n",
    (int)controller_index, platform.c_str());

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_platform(platform, args);
  }
}

void gams::controllers::HiveController::init_algorithm(
  size_t controller_index, algorithms::BaseAlgorithm * algorithm)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_algorithm:" \
    " deleting old algorithm at controller %d\n", (int)controller_index);

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_algorithm(algorithm);
  }
}


void gams::controllers::HiveController::init_platform(
  size_t controller_index, platforms::BasePlatform * platform)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_platform:" \
    " deleting old platform at controller %d\n", (int)controller_index);

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_platform(platform);
  }
}

#ifdef _GAMS_JAVA_

void gams::controllers::HiveController::init_algorithm(
  size_t controller_index, jobject algorithm)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_algorithm(java):" \
    " initializing java algorithm at controller %d\n", (int)controller_index);

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_algorithm(algorithm);
  }
}


void gams::controllers::HiveController::init_platform(
  size_t controller_index, jobject platform)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_platform(java):" \
    " initializing java platform at controller %d\n", (int)controller_index);

  if (controller_index < controllers_.size())
  {
    controllers_[controller_index]->init_platform(platform);
  }
}

#endif

void
gams::controllers::HiveController::init_vars(
const Integer & id,
const Integer & processes)
{
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::HiveController::init_vars:" \
    " %" PRId64 " id, %" PRId64 " processes\n", id, processes);

  offset_ = (size_t)id;

  // initialize the agents, swarm, and self variables
  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    controllers_[i]->init_vars(id + i, processes);
  }
}

void gams::controllers::HiveController::refresh_vars(
  bool init_non_self_vars)
{
  if (hive_ != 0)
  {
    std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

    for (size_t i = 0; i < controllers_.size(); ++i)
    {
      if (init_non_self_vars)
      {
        controllers_[i]->init_vars(
          (Integer)(i + offset_), (Integer)kbs.size());
      }
      else
      {
        // create the agent variables and set swarm size only
        controllers_[i]->init_vars((Integer)(i + offset_));
        kbs[i + offset_].set("swarm.size", (Integer)kbs.size());
      }
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::refresh_vars:" \
      " no hive has been set. Operation is a no-op.\n");
  }
}

gams::algorithms::BaseAlgorithm *
gams::controllers::HiveController::get_algorithm(size_t controller_index)
{
  gams::algorithms::BaseAlgorithm * result(0);

  if (controller_index < controllers_.size())
  {
    result = controllers_[controller_index]->get_algorithm();
  }

  return result;
}

gams::platforms::BasePlatform *
gams::controllers::HiveController::get_platform(size_t controller_index)
{
  gams::platforms::BasePlatform * result(0);

  if (controller_index < controllers_.size())
  {
    result = controllers_[controller_index]->get_platform();
  }

  return result;
}

madara::knowledge::KnowledgeBase
gams::controllers::HiveController::get_kb(size_t controller_index)
{
  if (hive_ != 0)
  {
    std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

    if (controller_index + offset_ < kbs.size())
    {
      return kbs[controller_index + offset_];
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::get_kb:" \
      " no hive has been set. Operation is a no-op.\n");
  }

  return madara::knowledge::KnowledgeBase();
}

size_t
gams::controllers::HiveController::get_num_controllers(void)
{
  return controllers_.size();
}

void gams::controllers::HiveController::resize(
  size_t init_id, size_t num_controllers,
  bool init_non_self_vars)
{
  if (hive_ != 0)
  {
    offset_ = init_id;
    size_t old_size = controllers_.size();

    if (offset_ + num_controllers > hive_->get_size())
    {
      num_controllers = hive_->get_size() - offset_;
    }

    if (num_controllers != old_size)
    {
      // if we're shrinking the num controllers, resize later
      if (old_size > num_controllers)
      {
        for (size_t i = num_controllers; i < old_size; ++i)
        {
          delete controllers_[i];
        }
      }

      // both conditions result in needing resizing here
      controllers_.resize(num_controllers);

      // if we're growing the num controllers, resize now
      if (old_size < num_controllers)
      {
        // handle case where resize is called after a default constructor
        if (old_size == 1 && settings_.shared_memory_transport)
        {
          delete controllers_[0];
          old_size = 0;
        }

        std::vector<knowledge::KnowledgeBase>& kbs = hive_->get_kbs();

        madara::transport::QoSTransportSettings transport_settings;
        for (size_t i = old_size; i < num_controllers; ++i)
        {
          controllers_[i] = new BaseController(kbs[i + offset_], settings_);

          if (init_non_self_vars)
          {
            // create the swarm variables with all agents populated
            controllers_[i]->init_vars((Integer)(i + offset_), (Integer)kbs.size());
          }
          else
          {
            // create the agent variables and set swarm size only
            controllers_[i]->init_vars((Integer)(i + offset_));
            kbs[i + offset_].set("swarm.size", (Integer)kbs.size());
          }
        } // for old_size < num_controllers
      }
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_WARNING,
      "gams::controllers::HiveController::resize:" \
      " no hive has been set. Operation is a no-op.\n");
  }
}

int
gams::controllers::HiveController::run(void)
{
  // check the debug levels and set accordingly
  if (settings_.madara_log_level >= 0)
  {
    madara::logger::global_logger->set_level(settings_.madara_log_level);
  }
  if (settings_.gams_log_level >= 0)
  {
    gams::loggers::global_logger->set_level(settings_.gams_log_level);
  }

  return run_hz(settings_.loop_hertz,
    settings_.run_time, settings_.send_hertz);
}

int
gams::controllers::HiveController::run(double loop_period,
  double max_runtime, double send_period)
{
  // return value
  int return_value(0);
  bool first_execute(true);

  // if user specified non-positive, then we are to use loop_period
  if (send_period <= 0)
  {
    send_period = loop_period;
  }

  madara::utility::TimeValue current = madara::utility::Clock::now();
  madara::utility::Duration loop_window =
    madara::utility::seconds_to_duration(loop_period);
  madara::utility::TimeValue next_loop = current + loop_window;
  madara::utility::TimeValue end_time = current +
    madara::utility::seconds_to_duration(max_runtime);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::run:" \
    " loop_period: %fs, max_runtime: %fs, send_period: %fs\n",
    loop_period, max_runtime, send_period);
  
  if (loop_period >= 0.0)
  {
    //unsigned int iterations = 0;
    while (first_execute || max_runtime < 0 || current < end_time)
    {
      // return value should be last return value of mape loop
      return_value = run_once();

      current = madara::utility::Clock::now();

      // check to see if we need to sleep for next loop epoch
      if (loop_period > 0.0 && (max_runtime < 0 || current < end_time))
      {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MINOR,
          "gams::controllers::BaseController::run:" \
          " sleeping until next epoch\n");

        std::this_thread::sleep_until(next_loop);

        current = madara::utility::Clock::now();
        while (next_loop <= current)
        {
          next_loop += loop_window;
        }
      }

      // run will always execute at least one time. Update flag for execution.
      if (first_execute)
        first_execute = false;

      current = madara::utility::Clock::now();
    }
  }

  return return_value;
}

int
gams::controllers::HiveController::run_once(void)
{
  // return value
  int return_value = 0;

  for (size_t i = 0; i < controllers_.size(); ++i)
  {
    return_value |= controllers_[i]->run_once();
  }

  return return_value;
}
