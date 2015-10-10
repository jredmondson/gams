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
#include "gams/platforms/PlatformFactory.h"
#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/loggers/GlobalLogger.h"

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
  madara::knowledge::KnowledgeBase & knowledge)
  : algorithm_ (0), knowledge_ (knowledge), platform_ (0),
  algorithm_factory_ (&knowledge, &sensors_, platform_, 0, &devices_),
  platform_factory_ (&knowledge, &sensors_, &platforms_, 0)
{
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
  this->platform_factory_.add (aliases, factory);
}

void gams::controllers::BaseController::add_algorithm_factory (
  const std::vector <std::string> & aliases,
  algorithms::AlgorithmFactory * factory)
{
  this->algorithm_factory_.add (aliases, factory);
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
   * Note that certain device variables like command are kept local only.
   * @see gams::variables::Device::init_vars
   * @see gams::variables::Swarm::init_vars
   **/

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::system_analyze:" \
    " checking device and swarm commands\n");

  if (self_.device.command != "")
  {
    madara::knowledge::KnowledgeVector args;

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Processing device command\n");

    // check for args
    self_.device.command_args.resize ();
    self_.device.command_args.copy_to (args);

    init_algorithm (self_.device.command.to_string (), args);

    self_.device.last_command = self_.device.command.to_string ();
    self_.device.last_command_args.resize (0);
    self_.device.command_args.transfer_to (self_.device.last_command_args);

    // reset the command
    self_.device.command = "";
    self_.device.command_args.resize (0);
  }
  else if (swarm_.command != "")
  {
    madara::knowledge::KnowledgeVector args;

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Processing swarm command\n");

    // check for args
    swarm_.command_args.resize ();
    swarm_.command_args.copy_to (args);

    init_algorithm (swarm_.command.to_string (), args);

    self_.device.last_command = swarm_.command.to_string ();
    self_.device.last_command_args.resize (0);
    swarm_.command_args.transfer_to (self_.device.last_command_args);

    // reset the command
    swarm_.command = "";
    swarm_.command_args.resize (0);
  }

  if (self_.device.madara_debug_level !=
    (Integer)madara::logger::global_logger->get_level ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Settings MADARA debug level to %d\n", (int)*self_.device.madara_debug_level);

    madara::logger::global_logger->set_level ((int)*self_.device.madara_debug_level);
  }

  if (self_.device.gams_debug_level !=
    (Integer)gams::loggers::global_logger->get_level ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::system_analyze:" \
      " Settings GAMS debug level to %d\n", (int)*self_.device.gams_debug_level);

    gams::loggers::global_logger->set_level ((int)*self_.device.gams_debug_level);
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

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::analyze:" \
    " calling system_analyze ()\n");
  return_value |= system_analyze ();

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
  knowledge_.lock ();

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
    "%s\n",
    knowledge_.debug_modifieds ().c_str ());

  // unlock the context to allow external updates
  knowledge_.unlock ();

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
gams::controllers::BaseController::run (double loop_period,
  double max_runtime, double send_period)
{
  // return value
  int return_value (0);
  bool first_execute (true);

  // for checking for potential user commands
  double loop_hz = 1.0 / loop_period;
  double send_hz = 1.0 / send_period;

  self_.device.loop_hz = loop_hz;
  self_.device.send_hz = send_hz;

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

      // grab current time
      current = ACE_OS::gettimeofday ();

      // run will always try to send at least once
      if (first_execute || current > send_next_epoch)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " sending updates\n");

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
        send_hz, self_.device.send_hz.to_double (), 0.001))
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " Changing send hertz from %.2f to %.2f\n", send_hz,
          self_.device.send_hz.to_double ());

        send_hz = self_.device.send_hz.to_double ();
        send_period = 1 / send_hz;
        send_poll_frequency.set (send_period);
        send_next_epoch = current + send_poll_frequency;
      }

      // if loop herz difference is more than .001 hz different, change epoch
      if (!madara::utility::approx_equal (
        loop_hz, self_.device.loop_hz.to_double (), 0.001))
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::controllers::BaseController::run:" \
          " Changing loop hertz from %.2f to %.2f\n", loop_hz,
          self_.device.loop_hz.to_double ());

        loop_hz = self_.device.loop_hz.to_double ();
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
        self_.device.loop_hz = loop_hz;
      }
    }
  }

  return return_value;
}

void
gams::controllers::BaseController::init_accent (const std::string & algorithm,
const madara::knowledge::KnowledgeVector & args)
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
    algorithms::ControllerAlgorithmFactory factory (&knowledge_, &sensors_,
      platform_, &self_, &devices_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_accent:" \
      " factory is creating accent %s\n", algorithm.c_str ());

    new_accent = factory.create (algorithm, args);

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
const std::string & algorithm, const madara::knowledge::KnowledgeVector & args)
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
    algorithms::ControllerAlgorithmFactory factory (&knowledge_, &sensors_,
      platform_, &self_, &devices_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm:" \
      " factory is creating algorithm %s\n", algorithm.c_str ());

    algorithm_ = factory.create (algorithm, args);

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
  const madara::knowledge::KnowledgeVector & args)
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
    platforms::ControllerPlatformFactory factory (&knowledge_, &sensors_, &platforms_, &self_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " factory is creating platform %s\n", platform.c_str ());

    platform_ = factory.create (platform, args);

    init_vars (*platform_);

    if (algorithm_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_platform:" \
        " algorithm is already initialized. Updating to new platform\n");

      algorithm_->set_platform (platform_);
    }
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

    if (algorithm_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_platform:" \
        " algorithm is already initialized. Updating to new platform\n");

      algorithm_->set_platform (platform_);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_platform:" \
      " platform was reset to none\n");
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

    if (algorithm_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_platform (java):" \
        " Algorithm exists. Updating its platform.\n");

      algorithm_->set_platform (platform_);
    }
  }
}

#endif

void
gams::controllers::BaseController::init_vars (
const Integer & id,
const Integer & processes)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " %" PRId64 " id, %" PRId64 " processes\n", id, processes);

  // initialize the devices, swarm, and self variables
  variables::init_vars (devices_, knowledge_, processes);
  swarm_.init_vars (knowledge_, processes);
  self_.init_vars (knowledge_, id);
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
}


void
gams::controllers::BaseController::init_vars (algorithms::BaseAlgorithm & algorithm)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::controllers::BaseController::init_vars:" \
    " initializing algorithm's vars\n");

  algorithm.devices_ = &devices_;
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
