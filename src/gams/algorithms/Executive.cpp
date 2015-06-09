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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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

/**
 * @file Executive.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Definitions for Executive class. A series of algorithms are executed. 
 **/

#include "gams/algorithms/Executive.h"

#include "gams/algorithms/Controller_Algorithm_Factory.h"

#include <iostream>

using std::cerr;
using std::endl;

gams::algorithms::Base_Algorithm *
gams::algorithms::Executive_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * devices)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::algorithms::Executive_Factory::create:" \
    " creating Executive with %d args\n", args.size ()));

  Base_Algorithm * result (0);
  
  if (knowledge && sensors && platform && self && (args.size () % 2 == 0))
    result = new Executive (args, knowledge, platform, sensors, self);

  return result;
}

gams::algorithms::Executive::Executive (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  platforms::Base_Platform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  Base_Algorithm (knowledge, platform, sensors, self), algo_ (0), 
  plan_index_ (-1), plan_ (), algo_factory_ (knowledge, sensors, platform, self)
{
  knowledge->print ();
  plan_.reserve (args.size () / 2);
  for (size_t i = 0; i < args.size(); i += 2)
  {
    Madara::Knowledge_Engine::Containers::Vector v;
    v.set_name (args[i + 1].to_string (), *knowledge);
    v.resize ();
  
    Madara::Knowledge_Vector a;
    v.copy_to (a);
  
    Algorithm_Init init (args[i].to_string (), a);

    plan_.push_back (init);

    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::algorithms::Executive::Executive:" \
      " queueing %s algorithm\n", init.algorithm.c_str ()));
  }
}

gams::algorithms::Executive::~Executive ()
{
  delete algo_;
}

void
gams::algorithms::Executive::operator= (Executive& rhs)
{
  if (this != &rhs)
  {
//    if (rhs.algo_ != 0)
//      this->algo_ = rhs.algo_/*->clone ()*/;
//    else
//      this->algo_ = 0;

    this->plan_index_ = rhs.plan_index_;
    //this->plan_ = rhs.plan_;
    this->algo_factory_ = rhs.algo_factory_;
  }
}

int
gams::algorithms::Executive::analyze (void)
{
  int ret_val (UNKNOWN);

  if (algo_ != 0)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::algorithms::Executive::analyze:" \
      " algo != 0\n"));
    ret_val = algo_->analyze ();
    if (ret_val == FINISHED)
    {
      delete algo_;
      algo_ = 0;
    }
  }

  if (algo_ == 0)
  {
    ++plan_index_;

    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::algorithms::Executive::analyze:" \
      " algo == 0, going to step %u\n", plan_index_));

    if (plan_index_ < plan_.size ())
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::algorithms::Executive::analyze:" \
        " creating algorithm %s\n", plan_[plan_index_].algorithm.c_str ()));

      algo_ = algo_factory_.create (plan_[plan_index_].algorithm, 
        plan_[plan_index_].args);
      ret_val = OK;
    }
    else
    {
      --plan_index_;  // to prevent plan_index_ from overflowing 
                      // and being a valid index again
      ret_val = FINISHED;
    }
  }
   
  return ret_val;
}
      
int
gams::algorithms::Executive::execute (void)
{
  int ret_val (0);
  if (algo_ != 0)
    ret_val = algo_->execute ();

  return ret_val;
}

int
gams::algorithms::Executive::plan (void)
{
  int ret_val (0);

  if (algo_ != 0)
    ret_val = algo_->plan ();

  return 0;
}

gams::algorithms::Executive::Algorithm_Init::Algorithm_Init () :
  algorithm (), args ()
{
}

gams::algorithms::Executive::Algorithm_Init::Algorithm_Init (
  const std::string& a, const Madara::Knowledge_Vector& v) :
  algorithm (a), args (v)
{
}
