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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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

#include "Base_Controller.h"

#include <iostream>
#include <sstream>

#include "ace/High_Res_Timer.h"
#include "ace/OS_NS_sys_time.h"
#include "madara/utility/Utility.h"
#include "gams/platforms/Platform_Factory.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/utility/Logging.h"

// Java-specific header includes
#ifdef _GAMS_JAVA_
#include "gams/algorithms/java/Java_Algorithm.h"
#include "gams/platforms/java/Java_Platform.h"
#endif

using std::cerr;
using std::endl;

typedef  Madara::Knowledge_Record::Integer  Integer;

gams::controllers::Base::Base (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge)
  : algorithm_ (0), knowledge_ (knowledge), platform_ (0)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::constructor:" \
    " default constructor called.\n"));
}

gams::controllers::Base::~Base ()
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::destructor:" \
    " deleting algorithm.\n"));
  delete algorithm_;

  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::destructor:" \
    " deleting platform.\n"));
  delete platform_;
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::destructor:" \
    " deleting accents.\n"));
  for (algorithms::Algorithms::iterator i = accents_.begin ();
       i != accents_.end (); ++i)
  {
    delete *i;
  }
}

int
gams::controllers::Base::monitor (void)
{
  int result (0);

  if (platform_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::monitor:" \
      " calling platform_->sense ()\n"));

    result = platform_->sense ();
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::monitor:" \
      " Platform undefined. Unable to call platform_->sense ()\n"));
  }

  return result;
}

int
gams::controllers::Base::system_analyze (void)
{
  int return_value (0);
  bool error (false);

  /**
   * note that certain device variables like command are kept local only.
   * @see gams::variables::Device::init_vars
   * @see gams::variables::Swarm::init_vars
   **/
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::system_analyze:" \
    " checking device and swarm commands\n"));

  if (this->self_.device.command == "land" || this->swarm_.command == "land")
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::system_analyze:" \
      " initializing land algorithm\n"));

    init_algorithm ("land");
  }

  else if (this->self_.device.command == "takeoff" || 
    this->swarm_.command == "takeoff")
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::system_analyze:" \
      " initializing takeoff algorithm\n"));

    init_algorithm ("takeoff");
  }

  else if (this->self_.device.command == "cover" ||
    this->swarm_.command == "cover")
  {
    Madara::Knowledge_Record type;
    Madara::Knowledge_Record area;
    
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::system_analyze:" \
      " cover algorithm requested\n"));

    if (self_.device.command == "cover")
    {
      self_.device.command_args.resize ();

      if (this->self_.device.command_args.size () == 2)
      {
        type = self_.device.command_args[0];
        area = self_.device.command_args[1];
      }
      else
      {
        error = true;
      }
    }
    else
    {
      swarm_.command_args.resize ();

      // check if proper number of arguments were given
      if (swarm_.command_args.size () == 2)
      {
        type = swarm_.command_args[0];
        area = swarm_.command_args[1];
      }
      else
      {
        error = true;
      }
    }

    if (error)
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " ERROR: Area coverage requested with improper args\n"));
    }
    else
    {
      std::stringstream error_buffer;
      if (!type.is_string_type ())
      {
        error_buffer << "ERROR: Area coverage type (first arg) == "
          << type <<
          ". Type should be a string.\n";

        error = true;

        GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
          DLINFO "gams::controllers::Base::system_analyze:" \
          " ERROR: %s\n",
          error_buffer.str ().c_str ()));
      }
      if (!area.is_string_type ())
      {
        error_buffer << "ERROR: Area coverage area (second arg) == "
          << area <<
          ". Area should be a string.\n";

        error = true;

        GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
          DLINFO "gams::controllers::Base::system_analyze:" \
          " ERROR: %s\n",
          error_buffer.str ().c_str ()));
      }

      knowledge_.print ();
    }

    // check if args were of the right type
    if (!error)
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " initializing cover algorithm\n"));

      delete algorithm_;
      algorithms::Factory factory (&knowledge_, &sensors_,
        platform_, &self_, &devices_);

      Madara::Knowledge_Vector args (1);
      args[0] = area;

      algorithm_ = factory.create (type.to_string (), args);
    }
  }
  
  else if (this->self_.device.command == "move" ||
    this->swarm_.command == "move")
  {
    Madara::Knowledge_Record arg1;
    Madara::Knowledge_Record arg2;
    utility::Position target;
    
    if (self_.device.command == "move")
    {
      self_.device.command_args.resize ();

      if (this->self_.device.command_args.size () == 2)
      {
        arg1 = self_.device.command_args[0];
        arg2 = self_.device.command_args[1];
      }
      else if (this->self_.device.command_args.size () == 1)
      {
        arg1 = self_.device.command_args[0];
      }
      else if (this->self_.device.command_args.size () == 0)
      {
        arg1 = "target";
      }
      else
      {
        error = true;
      }
    }
    else
    {
      swarm_.command_args.resize ();

      // check if proper number of arguments were given
      if (swarm_.command_args.size () == 2)
      {
        arg1 = swarm_.command_args[0];
        arg2 = swarm_.command_args[1];
      }
      else if (swarm_.command_args.size () == 1)
      {
        arg1 = swarm_.command_args[0];
      }
      else
      {
        error = true;
      }
    }

    if (error)
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " ERROR: move requested with improper args\n"));
    }
    else
    {
      delete algorithm_;
      algorithms::Factory factory (&knowledge_, &sensors_,
        platform_, &self_, &devices_);

      Madara::Knowledge_Vector args (2);
      args[0] = arg1;
      args[1] = arg2;
      
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " initializing move algorithm\n"));

      algorithm_ = factory.create ("move", args);
    }
  }
 
  else if (this->self_.device.command == "formation" ||
    this->swarm_.command == "formation")
  {
    Madara::Knowledge_Record target_id;
    Madara::Knowledge_Record cylindrical_offset;
    Madara::Knowledge_Record destination;
    Madara::Knowledge_Record members;
    Madara::Knowledge_Record modifier;

    if (self_.device.command == "formation")
    {
      self_.device.command_args.resize ();

      if (this->self_.device.command_args.size () == 4)
      {
        target_id = self_.device.command_args[0];
        cylindrical_offset = self_.device.command_args[1];
        destination = self_.device.command_args[2];
        members = self_.device.command_args[3];
      }
      else if (this->self_.device.command_args.size () == 5)
      {
        target_id = self_.device.command_args[0];
        cylindrical_offset = self_.device.command_args[1];
        destination = self_.device.command_args[2];
        members = self_.device.command_args[3];
        modifier = self_.device.command_args[4];
      }
      else
      {
        error = true;
      }
    }
    else // swarm command
    {
    }

    if (error)
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " ERROR: formation requested with improper args\n"));
    }
    else
    {
      delete algorithm_;
      algorithms::Factory factory (&knowledge_, &sensors_,
        platform_, &self_, &devices_);

      Madara::Knowledge_Vector args (5);
      args[0] = target_id;
      args[1] = cylindrical_offset;
      args[2] = destination;
      args[3] = members;
      args[4] = modifier;
      
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " initializing formation algorithm\n"));

      algorithm_ = factory.create ("formation", args);
    }
  }

  else if (this->self_.device.command == "formation coverage" ||
    this->swarm_.command == "formation coverage")
  {
//    Madara::Knowledge_Record head_id;
//    Madara::Knowledge_Record cylindrical_offset;
//    Madara::Knowledge_Record members;
//    Madara::Knowledge_Record modifier;
//    Madara::Knowledge_Record coverage;
//    Madara::Knowledge_Record cover_args;
//
//    if (self_.device.command == "formation coverage")
//    {
//      self_.device.command_args.resize ();
//
//      if (this->self_.device.command_args.size () == 4)
//      {
//        target_id = self_.device.command_args[0];
//        cylindrical_offset = self_.device.command_args[1];
//        destination = self_.device.command_args[2];
//        members = self_.device.command_args[3];
//      }
//      else if (this->self_.device.command_args.size () == 5)
//      {
//        target_id = self_.device.command_args[0];
//        cylindrical_offset = self_.device.command_args[1];
//        destination = self_.device.command_args[2];
//        members = self_.device.command_args[3];
//        modifier = self_.device.command_args[4];
//      }
//      else
//      {
//        error = true;
//      }
//    }
//    else // swarm command
//    {
//    }

    if (error)
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " ERROR: formation coverage requested with improper args\n"));
    }
    else
    {
      delete algorithm_;
      algorithms::Factory factory (&knowledge_, &sensors_,
        platform_, &self_, &devices_);

      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " initializing formation coverage algorithm\n"));

      self_.device.command_args.resize();
      Madara::Knowledge_Vector args (self_.device.command_args.size ());
      for (size_t i = 0; i < args.size (); ++i)
        args[i] = self_.device.command_args[i];

      algorithm_ = factory.create ("formation coverage", args);
    }
  }

  else if (this->self_.device.command == "follow")
  {
    Madara::Knowledge_Record target_id;
    Madara::Knowledge_Record delay;

    if (self_.device.command == "follow")
    {
      self_.device.command_args.resize ();

      if (this->self_.device.command_args.size () == 1)
      {
        target_id = self_.device.command_args[0];
        delay.set_value ((Madara::Knowledge_Record::Integer)5);
      }
      else if (this->self_.device.command_args.size () == 2)
      {
        target_id = self_.device.command_args[0];
        delay = self_.device.command_args[1];
      }
      else
      {
        error = true;
      }
    }

    if (error)
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " ERROR: follow requested with improper args\n"));
    }
    else
    {
      delete algorithm_;
      algorithms::Factory factory (&knowledge_, &sensors_,
        platform_, &self_, &devices_);

      Madara::Knowledge_Vector args (2);
      args[0] = target_id;
      args[1] = delay;

      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::system_analyze:" \
        " initializing follow algorithm\n"));

      algorithm_ = factory.create ("follow", args);
    }
  }
  
  self_.device.command = "";
  swarm_.command = "";
  self_.device.command_args.resize (0);
  swarm_.command_args.resize (0);

  return return_value;
}

int
gams::controllers::Base::analyze (void)
{
  int return_value (0);

  if (platform_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::analyze:" \
      " calling platform_->analyze ()\n"));

    return_value |= platform_->analyze ();
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::analyze:" \
      " Platform undefined. Unable to call platform_->analyze ()\n"));
  }
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::analyze:" \
    " calling system_analyze ()\n"));
  return_value |= system_analyze ();

  if (algorithm_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::analyze:" \
      " calling algorithm_->analyze ()\n"));

    return_value |= algorithm_->analyze ();
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::analyze:" \
      " Algorithm undefined. Unable to call algorithm_->analyze ()\n"));
  }


  if (accents_.size () > 0)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::analyze:" \
      " calling analyze on accents\n"));
    for (algorithms::Algorithms::iterator i = accents_.begin ();
      i != accents_.end (); ++i)
    {
      (*i)->analyze ();
    }
  }

  return return_value;
}

int
gams::controllers::Base::plan (void)
{
  int return_value (0);
  
  if (algorithm_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::plan:" \
      " calling algorithm_->plan ()\n"));

    return_value |= algorithm_->plan ();
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::plan:" \
      " Algorithm undefined. Unable to call algorithm_->plan ()\n"));
  }

  if (accents_.size () > 0)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::plan:" \
      " calling plan on accents\n"));

    for (algorithms::Algorithms::iterator i = accents_.begin ();
      i != accents_.end (); ++i)
    {
      (*i)->plan ();
    }
  }

  return return_value;
}

int
gams::controllers::Base::execute (void)
{
  int return_value (0);
  
  if (algorithm_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::execute:" \
      " calling algorithm_->execute ()\n"));

    return_value |= algorithm_->execute ();
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::execute:" \
      " Algorithm undefined. Unable to call algorithm_->execute ()\n"));
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
gams::controllers::Base::run (double loop_period,
  double max_runtime, double send_period)
{
  // return value
  int return_value (0);
  bool first_execute (true);

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
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::run:" \
    " loop_period: %ds, max_runtime: %ds, send_period: %ds\n",
    loop_period, max_runtime, send_period));

  if (loop_period >= 0.0)
  {
    max_wait.set (max_runtime);
    max_wait = current + max_wait;
    
    poll_frequency.set (loop_period);
    send_poll_frequency.set (send_period);
    next_epoch = current + poll_frequency;
    send_next_epoch = current;

    unsigned int iterations = 0;
    while (first_execute || max_runtime < 0 || current < max_wait)
    {
      // return value should be last return value of mape loop
      return_value = 0;
      
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::run:" \
        " calling monitor ()\n"));

      // lock the context from any external updates
      knowledge_.lock ();

      return_value |= monitor ();

      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::run:" \
        " calling analyze ()\n"));

      return_value |= analyze ();

      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::run:" \
        " calling plan ()\n"));

      return_value |= plan ();

      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::run:" \
        " calling execute ()\n"));

      return_value |= execute ();
    
      // unlock the context to allow external updates
      knowledge_.unlock ();

      // grab current time
      current = ACE_OS::gettimeofday ();
      
      // run will always try to send at least once
      if (first_execute || current > send_next_epoch)
      {
        GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
          DLINFO "gams::controllers::Base::run:" \
          " sending updates\n"));

        // send modified values through network
        knowledge_.send_modifieds();

        // setup the next send epoch
        while (send_next_epoch < current)
          send_next_epoch += send_poll_frequency;
      }

      // check to see if we need to sleep for next loop epoch
      if (loop_period > 0.0 && current < next_epoch)
      {
        GAMS_DEBUG (gams::utility::LOG_MINOR_EVENT, (LM_DEBUG, 
          DLINFO "gams::controllers::Base::run:" \
          " sleeping until next epoch\n"));

        Madara::Utility::sleep (next_epoch - current);  
      }
      
      // setup the next 
      next_epoch += poll_frequency;

      // run will always execute at least one time. Update flag for execution.
      if (first_execute)
        first_execute = false;
    }
  }

  return return_value;
}

void
gams::controllers::Base::init_accent (const std::string & algorithm,
  const Madara::Knowledge_Vector & args)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_accent:" \
    " initializing accent %s\n", algorithm.c_str ()));

  if (algorithm == "")
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_accent:" \
      " ERROR: accent name is null\n"));
  }
  else
  {
    // create new accent pointer and algorithm factory
    algorithms::Base * new_accent (0);
    algorithms::Factory factory (&knowledge_, &sensors_,
      platform_, &self_, &devices_);
    
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_accent:" \
      " factory is creating accent %s\n", algorithm.c_str ()));

    new_accent = factory.create (algorithm, args);

    if (new_accent)
    {
      accents_.push_back (new_accent);
    }
    else
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::init_accent:" \
        " ERROR: created accent is null.\n"));
    }
  }
}

void gams::controllers::Base::clear_accents (void)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::clear_accents:" \
    " deleting and clearing all accents\n"));

  for (unsigned int i = 0; i < accents_.size (); ++i)
  {
    delete accents_[i];
  }

  accents_.clear ();
}
void
gams::controllers::Base::init_algorithm (
const std::string & algorithm, const Madara::Knowledge_Vector & args)
{
  // initialize the algorithm
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_algorithm:" \
    " initializing algorithm %s\n", algorithm.c_str ()));

  if (algorithm == "")
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "Algorithm is empty.\n\n" \
      "SUPPORTED ALGORITHMS:\n" \
      "  bridge | bridging\n" \
      "  random area coverage\n" \
      "  priotized area coverage\n"
      ));
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_platform:" \
      " deleting old algorithm\n"));

    delete algorithm_;
    algorithms::Factory factory (&knowledge_, &sensors_,
      platform_, &self_, &devices_);
    
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_algorithm:" \
      " factory is creating algorithm %s\n", algorithm.c_str ()));

    algorithm_ = factory.create (algorithm, args);

    init_vars (*algorithm_);
  }
}

void
gams::controllers::Base::init_platform (
  const std::string & platform,
  const Madara::Knowledge_Vector & args)
{
  // initialize the platform
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_platform:" \
    " initializing algorithm %s\n", platform.c_str ()));

  if (platform == "")
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "Platform is empty.\n\n" \
      "SUPPORTED PLATFORMS:\n" \
      "  drone-rk\n" \
      "  vrep\n" \
      ));
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_platform:" \
      " deleting old platform\n"));

    delete platform_;
    platforms::Factory factory (&knowledge_, &sensors_, &platforms_, &self_);

    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_platform:" \
      " factory is creating platform %s\n", platform.c_str ()));

    platform_ = factory.create (platform);
    
    init_vars (*platform_);

    if (algorithm_)
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::init_platform:" \
        " algorithm is already initialized. Updating to new platform\n"));

      algorithm_->set_platform (platform_);
    }
  }
}

void gams::controllers::Base::init_algorithm (algorithms::Base * algorithm)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_algorithm:" \
    " deleting old algorithm\n"));

  delete algorithm_;
  algorithm_ = algorithm;
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_algorithm:" \
    " initializing vars in algorithm\n"));

  init_vars (*algorithm_);
}
      

void gams::controllers::Base::init_platform (platforms::Base * platform)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_platform:" \
    " deleting old platform\n"));

  delete platform_;
  platform_ = platform;

  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_platform:" \
    " initializing vars in platform\n"));

  init_vars (*platform_);

  if (algorithm_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_platform:" \
      " algorithm is already initialized. Updating to new platform\n"));

    algorithm_->set_platform (platform_);
  }
}

#ifdef _GAMS_JAVA_

void gams::controllers::Base::init_algorithm (jobject algorithm)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_algorithm (java):" \
    " deleting old algorithm\n"));

  delete algorithm_;

  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_algorithm (java):" \
    " creating new Java algorithm\n"));

  algorithm_ = new gams::algorithms::Java_Algorithm (algorithm);

  if (algorithm_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_algorithm (java):" \
      " initializing vars for algorithm\n"));

    init_vars (*algorithm_);
  }
}
      

void gams::controllers::Base::init_platform (jobject platform)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_platform (java):" \
    " deleting old platform\n"));

  delete platform_;

  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_platform (java):" \
    " creating new Java platform\n"));

  platform_ = new gams::platforms::Java_Platform (platform);

  if (platform_)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::controllers::Base::init_platform (java):" \
      " initializing vars for platform\n"));

    init_vars (*platform_);

    if (algorithm_)
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::controllers::Base::init_platform (java):" \
        " Algorithm exists. Updating its platform.\n"));

      algorithm_->set_platform (platform_);
    }
  }
}

#endif

void
gams::controllers::Base::init_vars (
  const Integer & id,
  const Integer & processes)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_vars:" \
    " %q id, %q processes\n", id, processes));

  // initialize the devices, swarm, and self variables
  variables::init_vars (devices_, knowledge_, processes);
  swarm_.init_vars (knowledge_, processes);
  self_.init_vars (knowledge_, id);
}

void
gams::controllers::Base::init_vars (platforms::Base & platform)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_vars:" \
    " initializing platform's vars\n"));

  platform.knowledge_ = &knowledge_;
  platform.self_ = &self_;
  platform.sensors_ = &sensors_;
}


void
gams::controllers::Base::init_vars (algorithms::Base & algorithm)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::controllers::Base::init_vars:" \
    " initializing algorithm's vars\n"));

  algorithm.devices_ = &devices_;
  algorithm.knowledge_ = &knowledge_;
  algorithm.platform_ = platform_;
  algorithm.self_ = &self_;
  algorithm.sensors_ = &sensors_;
}

gams::algorithms::Base *
gams::controllers::Base::get_algorithm (void)
{
  return algorithm_;
}
      
gams::platforms::Base *
gams::controllers::Base::get_platform (void)
{
  return platform_;
}
