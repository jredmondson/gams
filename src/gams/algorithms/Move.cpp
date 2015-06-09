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
#include "Move.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

using std::vector;
using std::stringstream;
using std::string;
using std::cerr;
using std::endl;

gams::algorithms::Base_Algorithm *
gams::algorithms::Move_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  Base_Algorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    if (args.size () == 1)
    {
      if (args[0].is_integer_type ())
      {
        vector<Madara::Knowledge_Record::Integer> targ (args[0].to_integers ());
        cerr << "vector size: " << targ.size () << endl;
        utility::Position target;
        if (targ.size () > 0)
        {
          target.x = targ[0];
          if (targ.size () > 1)
          {
            target.y = targ[1];
            if (targ.size () > 2)
              target.z = targ[2];
          }
        }
        result = new Move (target, knowledge, platform, sensors, self);
      }
      else
      {
        GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
          DLINFO "gams::algorithms::Move_Factory::create:" \
          " bad arguments"));
      }
    }
  }

  return result;
}

gams::algorithms::Move::Move (const utility::Position & target, 
  Madara::Knowledge_Engine::Knowledge_Base * knowledge, 
  platforms::Base_Platform * platform, variables::Sensors * sensors, 
  variables::Self * self) :
  Base_Algorithm (knowledge, platform, sensors, self), target_ (target), 
  mode_ (TARGET)
{
  cerr << "MOVE TARGET: " << target_.to_string () << endl;
}

gams::algorithms::Move::Move (
  const string & type,
  unsigned int max_executions,
  double max_execution_time,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base_Algorithm (knowledge, platform, sensors, self),
  end_time_ (ACE_OS::gettimeofday ()), max_execution_time_ (max_execution_time),
  max_executions_ (max_executions), type_ (type), target_ (), mode_ (TARGET)
{
  // init status vars
  status_.init_vars (*knowledge, "move");

//  if (max_executions > 0)
//    mode_ = EXECUTIONS;
//  else
//    mode_ = TIMED;
}

gams::algorithms::Move::Move (
  const string & type,
  const utility::Position & target,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base_Algorithm (knowledge, platform, sensors, self),
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
    this->end_time_ = rhs.end_time_;
    this->max_execution_time_ = rhs.max_execution_time_;
    this->max_executions_ = rhs.max_executions_;
    this->mode_ = rhs.mode_;
    this->target_ = rhs.target_;
    this->type_ = rhs.type_;

    this->Base_Algorithm::operator=(rhs);
  }
}

int
gams::algorithms::Move::analyze (void)
{
  int ret_val (UNKNOWN);
  if (mode_ == TARGET)
  {
    ret_val = OK;
    cerr << "position: " << platform_->get_position()->to_string() << endl;
    cerr << "target: " << target_.to_string () << endl;
    if (platform_->get_position ()->approximately_equal (target_, 2))
    {
      ret_val = FINISHED;
      cerr << "move finished" << endl;
    }
    else
    {
      cerr << "still moving" << endl;
    }
  }
  return ret_val;
}

int
gams::algorithms::Move::execute (void)
{
//  if (mode_ == EXECUTIONS)
//  {
//  }
//  else if (mode_ == TIMED)
//  {
//  }
  if (mode_ == TARGET)
    platform_->move(target_);

  ++executions_;
  return 0;
}

int
gams::algorithms::Move::plan (void)
{
  return 0;
}
