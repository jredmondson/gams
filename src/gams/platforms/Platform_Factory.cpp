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
#include "Platform_Factory.h"
#include "Printer_Platform.h"
#include "Null_Platform.h"

#ifdef _GAMS_DRONERK_
#include "dronerk/Drone_RK.h"
#endif

#ifdef _GAMS_VREP_
#include "gams/platforms/vrep/VREP_UAV.h"
#include "gams/platforms/vrep/VREP_Ant.h"
#endif

#include <string>

gams::platforms::Factory::Factory (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
: knowledge_ (knowledge), platforms_ (platforms), self_ (self),
  sensors_ (sensors)
{
}

gams::platforms::Factory::~Factory ()
{
}

gams::platforms::Base *
gams::platforms::Factory::create (const std::string & type)
{
  static const std::string default_multicast ("239.255.0.1:4150");
  static const std::string default_broadcast ("192.168.1.255:15000");
  if (type == "debug" || type == "printer" || type == "print")
  {
    if (knowledge_ && sensors_ && platforms_ && self_)
    {
      // add transport to knowledge base if necessary
      Madara::Transport::Settings settings = knowledge_->transport_settings ();
      if (settings.type == Madara::Transport::NO_TRANSPORT)
      {
        settings.type = Madara::Transport::MULTICAST;
        settings.hosts.push_back (default_multicast);
        knowledge_->attach_transport ("", settings);
        knowledge_->activate_transport ();
        knowledge_->apply_modified ();
      }
      return new Printer_Platform (knowledge_, sensors_, platforms_, self_);
    }
  }
  else if (type == "null")
  {
    return new Null_Platform (knowledge_, sensors_, platforms_, self_);
  }
#ifdef _GAMS_DRONERK_
  else if (type == "drone-rk" || type == "dronerk")
  {
    if (knowledge_ && sensors_ && platforms_ && self_)
    {
      // add transport to knowledge base if necessary
      Madara::Transport::Settings settings = knowledge_->transport_settings ();
      if (settings.type == Madara::Transport::NO_TRANSPORT)
      {
        settings.type = Madara::Transport::BROADCAST;
        settings.hosts.push_back (default_broadcast);
        knowledge_->attach_transport ("", settings);
        knowledge_->activate_transport ();
        knowledge_->apply_modified ();
      }

      return new Drone_RK (knowledge_, sensors_, platforms_, self_);
    }
  }
#endif
#ifdef _GAMS_VREP_
  else if (type == "vrep" || type == "vrep-uav")
  {
    if (knowledge_ && sensors_ && platforms_ && self_)
    {
      // add transport to knowledge base if necessary
      Madara::Transport::Settings settings = knowledge_->transport_settings ();
      if (settings.type == Madara::Transport::NO_TRANSPORT)
      {
        settings.type = Madara::Transport::MULTICAST;
        settings.hosts.push_back (default_multicast);
        knowledge_->attach_transport ("", settings);
        knowledge_->activate_transport ();
        knowledge_->apply_modified ();
      }

      VREP_UAV* ret = new VREP_UAV (knowledge_, sensors_, platforms_, self_);
      double move_speed = knowledge_->get (".vrep_uav_move_speed").to_double ();
      if (move_speed > 0)
        ret->set_move_speed (move_speed);
      return ret;
    }
  }
  else if (type == "vrep-ant")
  {
    if (knowledge_ && sensors_ && platforms_ && self_)
    {
      // add transport to knowledge base if necessary
      Madara::Transport::Settings settings = knowledge_->transport_settings ();
      if (settings.type == Madara::Transport::NO_TRANSPORT)
      {
        settings.type = Madara::Transport::MULTICAST;
        settings.hosts.push_back (default_multicast);
        knowledge_->attach_transport ("", settings);
        knowledge_->activate_transport ();
        knowledge_->apply_modified ();
      }

      VREP_Ant* ret = new VREP_Ant (knowledge_, sensors_, platforms_, self_);
      return ret;
    }
  }
#endif

  return 0;
}

void
gams::platforms::Factory::set_knowledge (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge)
{
  knowledge_ = knowledge;
}

void
gams::platforms::Factory::set_platforms (variables::Platforms * platforms)
{
  platforms_ = platforms;
}

void
gams::platforms::Factory::set_self (variables::Self * self)
{
  self_ = self;
}

void
gams::platforms::Factory::set_sensors (variables::Sensors * sensors)
{
  sensors_ = sensors;
}
