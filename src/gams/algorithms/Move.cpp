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
#include "Move.h"

#include <string>
#include <iostream>

using std::string;
using std::cerr;
using std::endl;

gams::algorithms::Move::Move (
  const string & type,
  unsigned int max_executions,
  double max_execution_time,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base (knowledge, platform, sensors, self),
  end_time_ (ACE_OS::gettimeofday ()), max_execution_time_ (max_execution_time),
  max_executions_ (max_executions), type_ (type)
{
  // init status vars
  status_.init_vars (*knowledge, "move");

  if (max_executions > 0)
    mode_ = EXECUTIONS;
  else
    mode_ = TIMED;
}

gams::algorithms::Move::Move (
  const string & type,
  const utility::Position & target,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base (knowledge, platform, sensors, self),
  end_time_ (ACE_OS::gettimeofday ()), max_execution_time_ (-1),
  max_executions_ (0), mode_ (TARGET), target_ (target), type_ (type)
{
  status_.init_vars (*knowledge, "move");
}

gams::algorithms::Move::~Move ()
{
}

void
gams::algorithms::Move::operator= (const Move & rhs)
{
  if (this != &rhs)
  {
    this->mode_ = rhs.mode_;
    this->target_ = rhs.target_;
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
    this->max_executions_ = rhs.max_executions_;
    this->max_execution_time_ = rhs.max_execution_time_;
    this->end_time_ = rhs.end_time_;
  }
}

int
gams::algorithms::Move::analyze (void)
{
  return 0;
}
      
int
gams::algorithms::Move::execute (void)
{
  if (mode_ == EXECUTIONS)
  {
  }
  else if (mode_ == TIMED)
  {

  }
  else if (mode_ == TARGET)
  {
  }

  ++executions_;

  if ((executions_ / 20) % 2 == 0)
    platform_->takeoff ();
  else
    platform_->land ();

  return 0;
}

int
gams::algorithms::Move::plan (void)
{
  return 0;
}
