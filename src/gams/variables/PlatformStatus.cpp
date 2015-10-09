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
#include "gams/variables/PlatformStatus.h"

#include <string>

using std::string;

typedef  madara::KnowledgeRecord::Integer  Integer;

gams::variables::PlatformStatus::PlatformStatus ()
{
}

gams::variables::PlatformStatus::~PlatformStatus ()
{
}

void
gams::variables::PlatformStatus::operator= (const PlatformStatus & rhs)
{
  if (this != &rhs)
  {
    this->name = rhs.name;
    this->ok = rhs.ok;
    this->waiting = rhs.waiting;
    this->deadlocked = rhs.deadlocked;
    this->failed = rhs.failed;
    this->moving = rhs.moving;
    this->reduced_sensing = rhs.reduced_sensing;
    this->reduced_movement = rhs.reduced_movement;
    this->communication_available = rhs.communication_available;
    this->sensors_available = rhs.sensors_available;
    this->movement_available = rhs.movement_available;
    this->gps_spoofed = rhs.gps_spoofed;
  }
}

void
gams::variables::PlatformStatus::init_vars (
  madara::knowledge::KnowledgeBase & knowledge,
  const std::string & new_name)
{
  name = new_name;
  string prefix (make_variable_prefix ());

  // initialize the variable containers
  this->ok.set_name (prefix + ".ok", knowledge);
  this->waiting.set_name (prefix + ".waiting", knowledge);
  this->deadlocked.set_name (prefix + ".deadlocked", knowledge);
  this->failed.set_name (prefix + ".failed", knowledge);
  this->moving.set_name (prefix + ".moving", knowledge);
  this->reduced_sensing.set_name (prefix + ".reduced_sensing", knowledge);
  this->reduced_movement.set_name (prefix + ".reduced_movement", knowledge);
  this->communication_available.set_name (
    prefix + ".communication_available", knowledge);
  this->sensors_available.set_name (prefix + ".sensors_available", knowledge);
  this->movement_available.set_name (
    prefix + ".movement_available", knowledge);
  this->gps_spoofed.set_name (prefix + ".gps_spoofed", knowledge);

  init_variable_values ();
}

void
gams::variables::PlatformStatus::init_vars (
  madara::knowledge::Variables & knowledge,
  const std::string & new_name)
{
  name = new_name;
  string prefix (make_variable_prefix ());
  
  // initialize the variable containers
  this->ok.set_name (prefix + ".ok", knowledge);
  this->waiting.set_name (prefix + ".waiting", knowledge);
  this->deadlocked.set_name (prefix + ".deadlocked", knowledge);
  this->failed.set_name (prefix + ".failed", knowledge);
  this->moving.set_name (prefix + ".moving", knowledge);
  this->reduced_sensing.set_name (prefix + ".reduced_sensing", knowledge);
  this->reduced_movement.set_name (prefix + ".reduced_movement", knowledge);
  this->communication_available.set_name (
    prefix + ".communication_available", knowledge);
  this->sensors_available.set_name (prefix + ".sensors_available", knowledge);
  this->movement_available.set_name (
    prefix + ".movement_available", knowledge);
  this->gps_spoofed.set_name (prefix + ".gps_spoofed", knowledge);

  init_variable_values ();
}

string
gams::variables::PlatformStatus::make_variable_prefix () const
{
  std::stringstream buffer;
  buffer << ".platform.";
  buffer << name;
  return buffer.str ();
}

void
gams::variables::PlatformStatus::init_variable_values ()
{
  ok = 1;
  waiting = 0;
  deadlocked = 0;
  failed = 0;
  moving = 0;
  reduced_sensing = 0;
  reduced_movement = 0;
  communication_available = 0;
  sensors_available = 0;
  movement_available = 0;
  gps_spoofed = 0;
}
