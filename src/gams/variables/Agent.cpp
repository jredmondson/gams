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
#include "Agent.h"
#include "gams/loggers/GlobalLogger.h"

#include <string>

using std::string;

gams::variables::Agent::Agent ()
{
}

gams::variables::Agent::~Agent ()
{
}

void
gams::variables::Agent::operator= (const Agent & agent)
{
  if (this != &agent)
  {
    this->algorithm = agent.algorithm;
    this->algorithm_id = agent.algorithm_id;
    this->algorithm_changed = agent.algorithm_changed;
    this->algorithm_args = agent.algorithm_args;
    this->battery_remaining = agent.battery_remaining;
    this->bridge_id = agent.bridge_id;
    this->coverage_type = agent.coverage_type;
    this->is_mobile = agent.is_mobile;
    this->location = agent.location;
    this->orientation = agent.orientation;
    this->desired_altitude = agent.desired_altitude;
    this->source = agent.source;
    this->dest = agent.dest;
    this->home = agent.home;
    this->min_alt = agent.min_alt;
    this->next_coverage_type = agent.next_coverage_type;
    this->search_area_id = agent.search_area_id;
    this->temperature = agent.temperature;
    this->last_algorithm = agent.last_algorithm;
    this->last_algorithm_id = agent.last_algorithm_id;
    this->last_algorithm_args = agent.last_algorithm_args;
    this->accents = agent.accents;
    this->madara_debug_level = agent.madara_debug_level;
    this->gams_debug_level = agent.gams_debug_level;
    this->loop_hz = agent.loop_hz;
    this->send_hz = agent.send_hz;
    this->prefix = agent.prefix;
  }
}

void
gams::variables::Agent::init_vars (
  madara::knowledge::KnowledgeBase & knowledge,
  const std::string & prefix)
{
  // initialize the variable containers
  min_alt.set_name (prefix + ".min_alt", knowledge);
  location.set_name (prefix + ".location", knowledge);
  orientation.set_name (prefix + ".orientation", knowledge);
  desired_altitude.set_name (prefix + ".desired_altitude", knowledge);
  is_mobile.set_name (prefix + ".mobile", knowledge);
  battery_remaining.set_name (prefix + ".battery", knowledge);
  bridge_id.set_name (prefix + ".bridge_id", knowledge);
  coverage_type.set_name (prefix + ".area_coverage_type", knowledge);
  next_coverage_type.set_name (prefix + ".next_area_coverage_type",
    knowledge);
  search_area_id.set_name (prefix + ".search_area_id", knowledge);
  algorithm.set_name (prefix + ".algorithm", knowledge);
  algorithm_id.set_name (prefix + ".algorithm.id", knowledge);
  algorithm_changed.set_name (prefix + ".algorithm.changed", knowledge);
  algorithm_args.set_name (prefix + ".algorithm.args", knowledge);
  last_algorithm.set_name (prefix + ".algorithm.last", knowledge);
  last_algorithm_id.set_name (prefix + ".algorithm.last.id", knowledge);
  last_algorithm_args.set_name (prefix + ".algorithm.last.args", knowledge);
  variables::init_vars (accents, knowledge, prefix);
  home.set_name (prefix + ".home", knowledge);
  source.set_name (prefix + ".source", knowledge);
  dest.set_name (prefix + ".dest", knowledge);
  temperature.set_name (prefix + ".temperature", knowledge);
  madara_debug_level.set_name (prefix + ".madara_debug_level", knowledge);
  gams_debug_level.set_name (prefix + ".gams_debug_level", knowledge);
  loop_hz.set_name (prefix + ".loop_hz", knowledge);
  send_hz.set_name (prefix + ".send_hz", knowledge);

  this->prefix = prefix;

  // init settings
  init_variable_settings ();
}

void
gams::variables::Agent::init_vars (
  madara::knowledge::KnowledgeBase & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& id)
{
  // create the agent name string identifier ('agent.{id}')
  prefix = make_variable_name (id);

  init_vars (knowledge, prefix);
}

void
gams::variables::Agent::init_vars (
  madara::knowledge::Variables & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& id)
{
  // create the agent name string identifier ('agent.{id}')
  string agent_name (make_variable_name (id));
  string local_agent_name ("." + agent_name);

  prefix = agent_name;

  // initialize the variable containers
  min_alt.set_name (agent_name + ".min_alt", knowledge);
  location.set_name (agent_name + ".location", knowledge, 3);
  orientation.set_name (agent_name + ".orientation", knowledge, 3);
  desired_altitude.set_name (agent_name + ".desired_altitude", knowledge);
  is_mobile.set_name (agent_name + ".mobile", knowledge);
  battery_remaining.set_name (agent_name + ".battery", knowledge);
  bridge_id.set_name (agent_name + ".bridge_id", knowledge);
  coverage_type.set_name (agent_name + ".area_coverage_type", knowledge);
  next_coverage_type.set_name (agent_name + ".next_area_coverage_type",
    knowledge);
  search_area_id.set_name (agent_name + ".search_area_id", knowledge);
  algorithm.set_name (agent_name + ".algorithm", knowledge);
  algorithm_id.set_name (agent_name + ".algorithm.id", knowledge);
  algorithm_changed.set_name (agent_name + ".algorithm.changed", knowledge);
  algorithm_args.set_name (agent_name + ".algorithm.args", knowledge);
  last_algorithm.set_name (agent_name + ".algorithm.last", knowledge);
  last_algorithm_id.set_name (agent_name + ".algorithm.last.id", knowledge);
  last_algorithm_args.set_name (agent_name + ".algorithm.last.args", knowledge);
  variables::init_vars (accents, knowledge, agent_name);
  home.set_name (agent_name + ".home", knowledge);
  source.set_name (agent_name + ".source", knowledge);
  dest.set_name (agent_name + ".dest", knowledge);
  temperature.set_name (agent_name + ".temperature", knowledge);
  madara_debug_level.set_name (agent_name + ".madara_debug_level", knowledge);
  gams_debug_level.set_name (agent_name + ".gams_debug_level", knowledge);
  loop_hz.set_name (agent_name + ".loop_hz", knowledge);
  send_hz.set_name (agent_name + ".send_hz", knowledge);

  // init settings
  init_variable_settings ();

  madara_debug_level = madara::logger::global_logger->get_level ();
  gams_debug_level = gams::loggers::global_logger->get_level ();
}

void gams::variables::init_vars (Agents & variables,
  madara::knowledge::KnowledgeBase & knowledge,
  const madara::knowledge::KnowledgeRecord::Integer& processes)
{
  madara::knowledge::KnowledgeRecord::Integer limit = processes;
  if (processes >= 0)
  {
    variables.resize (processes);
  }
  else
  {
    limit = knowledge.get ("agent.size").to_integer ();
  }

  for (unsigned int i = 0; i < limit; ++i)
  {
    variables[i].init_vars (knowledge, i);
  }
}

void gams::variables::init_vars (Agents & variables,
  madara::knowledge::KnowledgeBase & knowledge,
  const groups::GroupBase & group)
{
  // get the member identifiers
  groups::AgentVector members;
  group.get_members (members);

  variables.resize (members.size ());

  for (unsigned int i = 0; i < members.size (); ++i)
  {
    variables[i].init_vars (knowledge, members[i]);
  } 
}

string
gams::variables::Agent::make_variable_name (
  const madara::knowledge::KnowledgeRecord::Integer& id)
{
  std::stringstream buffer;
  buffer << "agent.";
  buffer << id;
  return buffer.str ();
}

void
gams::variables::Agent::init_variable_settings ()
{
  // keep certain varaible changes as local only
  madara::knowledge::KnowledgeUpdateSettings keep_local (true);
  algorithm.set_settings (keep_local);
  algorithm_args.set_settings (keep_local);
  madara_debug_level.set_settings (keep_local);
  gams_debug_level.set_settings (keep_local);
}
