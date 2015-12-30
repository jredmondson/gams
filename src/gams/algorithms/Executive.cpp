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

#include "gams/algorithms/ControllerAlgorithmFactory.h"
#include "gams/loggers/GlobalLogger.h"

#include <iostream>

#include "gams/utility/ArgumentParser.h"

using std::cerr;
using std::endl;

gams::algorithms::BaseAlgorithm *
gams::algorithms::ExecutiveFactory::create (
  const madara::knowledge::KnowledgeMap & map,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::algorithms::ExecutiveFactory::create:" \
    " creating Executive with %d args\n", map.size ());

  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self && agents)
  {
    // Use a dumb workaround for now; TODO: convert this algo to use the map
    using namespace madara::knowledge;
    KnowledgeVector args(utility::kmap2kvec(map));

    if (args.size () % 2 == 0)
    {
      result = new Executive (args, knowledge, platform, sensors, self, agents);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::ExecutiveFactory::create:" \
        " invalid number of args, must be even\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::ExecutiveFactory::create:" \
      " knowledge, sensors, platform, self, or agents are invalid\n");
  }

  return result;
}

gams::algorithms::Executive::Executive (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents), algo_ (0), 
  plan_index_ (-1), plan_ (),
  algo_factory_ (knowledge, sensors, platform, self, agents)
{
  status_.init_vars (*knowledge, "executive", self->id.to_integer ());
  status_.init_variable_values ();

  plan_.reserve (args.size () / 2);
  for (size_t i = 0; i < args.size(); i += 2)
  {
    madara::knowledge::containers::Vector v;
    v.set_name (args[i + 1].to_string (), *knowledge);
    v.resize ();
  
    madara::knowledge::KnowledgeMap a;
    for(size_t j = 0; j < v.size(); ++j)
    {
      std::stringstream s;
      s << j;
      a[s.str()] = v[j];
    }
    //v.copy_to (a);
  
    AlgorithmInit init (args[i].to_string (), a);

    plan_.push_back (init);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::algorithms::Executive::Executive:" \
      " queueing %s algorithm\n", init.algorithm.c_str ());
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
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::algorithms::Executive::analyze:" \
      " algo != 0\n");

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

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executive::analyze:" \
      " algo == 0, going to step %u\n", plan_index_);

    if (plan_index_ < plan_.size ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::algorithms::Executive::analyze:" \
        " creating algorithm %s\n", plan_[plan_index_].algorithm.c_str ());

      algo_ = algo_factory_.create (plan_[plan_index_].algorithm, 
        plan_[plan_index_].args);
      ret_val = OK;
    }
    else
    {
      --plan_index_;  // to prevent plan_index_ from overflowing 
                      // and being a valid index again
      ret_val = FINISHED;
      status_.finished = 1;
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

  return ret_val;
}

gams::algorithms::Executive::AlgorithmInit::AlgorithmInit () :
  algorithm (), args ()
{
}

gams::algorithms::Executive::AlgorithmInit::AlgorithmInit (
  const std::string& a, const madara::knowledge::KnowledgeMap& v) :
  algorithm (a), args (v)
{
}
