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

#include "Counter_Algorithm.h"

#include <iostream>

gams::algorithms::Counter_Algorithm::Counter_Algorithm (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge)
  : Base_Algorithm (&knowledge),
  enable_analyze_counter (false),
  enable_execute_counter (false),
  enable_plan_counter (false)
{
  status_.init_vars (knowledge, "counter", 0);
  analyze_counter.set_name (".algorithm_analyzes", knowledge);
  execute_counter.set_name (".algorithm_executes", knowledge);
  plan_counter.set_name (".algorithm_plans", knowledge);
}

gams::algorithms::Counter_Algorithm::~Counter_Algorithm ()
{
}

void
gams::algorithms::Counter_Algorithm::operator= (const Counter_Algorithm & rhs)
{
  if (this != &rhs)
  {
    this->platform_ = rhs.platform_;
    this->sensors_ = rhs.sensors_;
    this->self_ = rhs.self_;
    this->status_ = rhs.status_;
    this->analyze_counter = rhs.analyze_counter;
    this->execute_counter = rhs.execute_counter;
    this->plan_counter = rhs.plan_counter;
    this->enable_analyze_counter = rhs.enable_analyze_counter;
    this->enable_execute_counter = rhs.enable_execute_counter;
    this->enable_plan_counter = rhs.enable_plan_counter;
  }
}


int
gams::algorithms::Counter_Algorithm::analyze (void)
{
  ++loops;
  if (enable_analyze_counter)
    ++analyze_counter;

  return 0;
}
      

int
gams::algorithms::Counter_Algorithm::execute (void)
{
  if (enable_execute_counter)
    ++execute_counter;

  return 0;
}


int
gams::algorithms::Counter_Algorithm::plan (void)
{
  if (enable_plan_counter)
    ++plan_counter;

  return 0;
}

void
gams::algorithms::Counter_Algorithm::reset_counters (void)
{
  loops = 0;
  analyze_counter = 0;
  execute_counter = 0;
  plan_counter = 0;
}
      
void
gams::algorithms::Counter_Algorithm::enable_counters (void)
{
  this->enable_analyze_counter = true;
  this->enable_execute_counter = true;
  this->enable_plan_counter = true;
}
      
void
gams::algorithms::Counter_Algorithm::disable_counters (void)
{
  this->enable_analyze_counter = false;
  this->enable_execute_counter = false;
  this->enable_plan_counter = false;
}
