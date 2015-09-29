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
#include "Device.h"
#include "gams/loggers/Global_Logger.h"

#include <string>

using std::string;

gams::variables::Device::Device ()
{
}

gams::variables::Device::~Device ()
{
}

void
gams::variables::Device::operator= (const Device & device)
{
  if (this != &device)
  {
    this->battery_remaining = device.battery_remaining;
    this->bridge_id = device.bridge_id;
    this->coverage_type = device.coverage_type;
    this->is_mobile = device.is_mobile;
    this->location = device.location;
    this->desired_altitude = device.desired_altitude;
    this->source = device.source;
    this->dest = device.dest;
    this->home = device.home;
    this->min_alt = device.min_alt;
    this->next_coverage_type = device.next_coverage_type;
    this->search_area_id = device.search_area_id;
    this->temperature = device.temperature;
    this->command = device.command;
    this->command_args = device.command_args;
    this->last_command = device.last_command;
    this->last_command_args = device.last_command_args;
    this->accents = device.accents;
    this->madara_debug_level = device.madara_debug_level;
    this->gams_debug_level = device.gams_debug_level;
    this->loop_hz = device.loop_hz;
    this->send_hz = device.send_hz;
  }
}

void
gams::variables::Device::init_vars (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const Madara::Knowledge_Record::Integer& id)
{
  // create the device name string identifier ('device.{id}')
  string device_name (make_variable_name (id));
  string local_device_name ("." + device_name);

  // initialize the variable containers
  min_alt.set_name (device_name + ".min_alt", knowledge);
  location.set_name (device_name + ".location", knowledge);
  desired_altitude.set_name (device_name + ".desired_altitude", knowledge);
  is_mobile.set_name (device_name + ".mobile", knowledge);
  battery_remaining.set_name (device_name + ".battery", knowledge);
  bridge_id.set_name (device_name + ".bridge_id", knowledge);
  coverage_type.set_name (device_name + ".area_coverage_type", knowledge);
  next_coverage_type.set_name (device_name + ".next_area_coverage_type",
    knowledge);
  search_area_id.set_name (device_name + ".search_area_id", knowledge);
  command.set_name (device_name + ".command", knowledge);
  last_command.set_name (device_name + ".last_command", knowledge);
  variables::init_vars (accents, knowledge, device_name);
  home.set_name (device_name + ".home", knowledge);
  source.set_name (device_name + ".source", knowledge);
  dest.set_name (device_name + ".dest", knowledge);
  command_args.set_name (device_name + ".command", knowledge);
  last_command_args.set_name (device_name + ".last_command", knowledge);
  temperature.set_name (device_name + ".temperature", knowledge);
  madara_debug_level.set_name (device_name + ".madara_debug_level", knowledge);
  gams_debug_level.set_name (device_name + ".gams_debug_level", knowledge);
  loop_hz.set_name (device_name + ".loop_hz", knowledge);
  send_hz.set_name (device_name + ".send_hz", knowledge);

  // init settings
  init_variable_settings ();

  madara_debug_level = Madara::Logger::global_logger->get_level ();
  gams_debug_level = gams::loggers::global_logger->get_level ();
}

void
gams::variables::Device::init_vars (
  Madara::Knowledge_Engine::Variables & knowledge,
  const Madara::Knowledge_Record::Integer& id)
{
  // create the device name string identifier ('device.{id}')
  string device_name (make_variable_name (id));
  string local_device_name ("." + device_name);

  // initialize the variable containers
  min_alt.set_name (device_name + ".min_alt", knowledge);
  location.set_name (device_name + ".location", knowledge, 3);
  desired_altitude.set_name (device_name + ".desired_altitude", knowledge);
  is_mobile.set_name (device_name + ".mobile", knowledge);
  battery_remaining.set_name (device_name + ".battery", knowledge);
  bridge_id.set_name (device_name + ".bridge_id", knowledge);
  coverage_type.set_name (device_name + ".area_coverage_type", knowledge);
  next_coverage_type.set_name (device_name + ".next_area_coverage_type",
    knowledge);
  search_area_id.set_name (device_name + ".search_area_id", knowledge);
  command.set_name (device_name + ".command", knowledge);
  last_command.set_name (device_name + ".last_command", knowledge);
  variables::init_vars (accents, knowledge, device_name);
  home.set_name (device_name + ".home", knowledge);
  source.set_name (device_name + ".source", knowledge);
  dest.set_name (device_name + ".dest", knowledge);
  command_args.set_name (device_name + ".command", knowledge);
  last_command_args.set_name (device_name + ".last_command", knowledge);
  temperature.set_name (device_name + ".temperature", knowledge);
  madara_debug_level.set_name (device_name + ".madara_debug_level", knowledge);
  gams_debug_level.set_name (device_name + ".gams_debug_level", knowledge);
  loop_hz.set_name (device_name + ".loop_hz", knowledge);
  send_hz.set_name (device_name + ".send_hz", knowledge);

  // init settings
  init_variable_settings ();

  madara_debug_level = Madara::Logger::global_logger->get_level ();
  gams_debug_level = gams::loggers::global_logger->get_level ();
}

void gams::variables::init_vars (Devices & variables,
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const Madara::Knowledge_Record::Integer& processes)
{
  Madara::Knowledge_Record::Integer limit = processes;
  if (processes >= 0)
  {
    variables.resize (processes);
  }
  else
  {
    limit = knowledge.get ("device.size").to_integer ();
  }

  for (unsigned int i = 0; i < limit; ++i)
  {
    variables[i].init_vars (knowledge, i);
  }
}

string
gams::variables::Device::make_variable_name (
  const Madara::Knowledge_Record::Integer& id)
{
  std::stringstream buffer;
  buffer << "device.";
  buffer << id;
  return buffer.str ();
}

void
gams::variables::Device::init_variable_settings ()
{
  // keep certain varaible changes as local only
  Madara::Knowledge_Engine::Knowledge_Update_Settings keep_local (true);
  command.set_settings (keep_local);
  command_args.set_settings (keep_local);
  //madara_debug_level.set_settings (keep_local);
  //gams_debug_level.set_settings (keep_local);
}
