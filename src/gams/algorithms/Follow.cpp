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
 */

/**
 * @file Follow.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the Follow algorithm
 */

#include "gams/algorithms/Follow.h"

#include <sstream>
#include <iostream>
#include <limits.h>
using std::cerr;
using std::endl;

#include "gams/utility/GPS_Position.h"

using std::stringstream;

gams::algorithms::Base_Algorithm *
gams::algorithms::Follow_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  Base_Algorithm * result (0);
  
  if (knowledge && sensors && self && 
      args.size () == 2 &&
      args[0].is_integer_type () && args[1].is_integer_type ())
  {
    result = new Follow (
      args[0] /*follow target*/,
      args[1] /*timestep delay*/,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Follow::Follow (
  const Madara::Knowledge_Record& id,
  const Madara::Knowledge_Record& delay,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform, variables::Sensors * sensors,
  variables::Self * self) :
  Base_Algorithm (knowledge, platform, sensors, self), next_position_ (DBL_MAX),
  delay_ (delay.to_integer ())
{
  stringstream location_string;
  location_string << "device." << id.to_integer () << ".location";
  target_location_.set_name (location_string.str (), *knowledge, 3);
}

gams::algorithms::Follow::~Follow ()
{
}

void
gams::algorithms::Follow::operator= (const Follow & rhs)
{
  if (this != &rhs)
  {
    this->Base_Algorithm::operator= (rhs);
    this->target_location_ = rhs.target_location_;
    this->next_position_ = rhs.next_position_;
    this->previous_locations_ = rhs.previous_locations_;
    this->delay_ = rhs.delay_;
  }
}

/**
 * The agent gets the target's location from the database and adds it to the
 * queue of positions being stored.
 */
int
gams::algorithms::Follow::analyze (void)
{
  static utility::GPS_Position prev;
  utility::GPS_Position current;
  current.from_container (target_location_);

  // if target agent has moved
  if (current.distance_to (prev) > 1.0)
  {
    previous_locations_.push (current);
    prev = current;
  }

  ++executions_;
  return 0;
}
      
/**
 * Move to next location if next_position_ is valid
 */
int
gams::algorithms::Follow::execute (void)
{
  if (next_position_.latitude () != DBL_MAX)
    platform_->move (next_position_);
  
  return 0;
}

/**
 * Store locations in the queue up to the delay amount
 */
int
gams::algorithms::Follow::plan (void)
{
  if (previous_locations_.size () == delay_)
  {
    next_position_ = previous_locations_.front ();
    previous_locations_.pop ();
  }
  return 0;
}
