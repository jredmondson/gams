/**
 * Copyright (c) 2016 Carnegie Mellon University. All Rights Reserved.
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
 * @file Executor.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Implementation of the Executor algorithm. The Executor algorithm executes
 * other algorithms.
 **/

#include <sstream>

#include "ace/OS_NS_sys_time.h"

#include "gams/algorithms/Executor.h"

#include "gams/algorithms/AlgorithmFactoryRepository.h"
#include "gams/loggers/GlobalLogger.h"

namespace knowledge = madara::knowledge;
namespace containers = knowledge::containers;

typedef knowledge::KnowledgeRecord KnowledgeRecord;
typedef KnowledgeRecord::Integer   Integer;
typedef knowledge::KnowledgeMap    KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::ExecutorFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::algorithms::ExecutorFactory::create:" \
     " creating Executor with %d args\n", args.size ());

  BaseAlgorithm * result (0);
  
  if (knowledge && sensors && platform && self)
  {
    int repeat = 0;
    AlgorithmMetaDatas algorithms;

    KnowledgeMap::const_iterator size_found = args.find ("size");

    if (size_found != args.end ())
    {
      int size = (int) size_found->second.to_integer ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::ExecutorFactory::create:" \
        " Number of algorithms is %d\n", size);

      if (size > 0)
      {
        algorithms.resize (size);

        KnowledgeMap::const_iterator next = args.find ("0.algorithm");

        std::string alg_prefix = "0.algorithm";
        std::string args_prefix = "0.algorithm.args";

        /**
         * iterate over all keys from 0.algorithm to size
         **/
        for (; next != size_found; ++next)
        {
          if (next->first.size () > 0)
          {
            size_t last;

            // check for algorithm prefix of a number
            for (last = 0; last < next->first.size () &&
              next->first[last] >= '0' && next->first[last] <= '9'; ++last)
            {
            };

            // if this is an algorithm prefix
            if (last > 0)
            {
              std::string str_index = next->first.substr (0, last);
              size_t index = (size_t)(KnowledgeRecord (str_index).to_integer ());
              const std::string alg_prefix = str_index + ".algorithm";
              const std::string args_prefix = alg_prefix + ".args.";
              const std::string precond_prefix = str_index + ".precond";

              // we have an algorithm definition
              if (next->first == alg_prefix)
              {
                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_MINOR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Algorithm %d id set to %s\n",
                  (int)index, next->second.to_string ().c_str ());

                algorithms[index].id = next->second.to_string ();
              } // end algorithm define
              else if (madara::utility::begins_with (next->first, args_prefix))
              {
                std::string arg = madara::utility::strip_prefix (
                  next->first, args_prefix);

                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_MINOR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Algorithm %d added arg %s = %s\n",
                  (int)index, arg.c_str (), next->second.to_string ().c_str ());

                algorithms[index].args[arg] = next->second;
              } // end algorithm arg define
              else if (next->first == precond_prefix)
              {
                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_MINOR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Algorithm %d added precond %s\n",
                  (int)index, next->second.to_string ().c_str ());

                algorithms[index].precond = next->second.to_string ();
              }
              else if (madara::utility::ends_with (next->first, "max_time"))
              {
                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_MINOR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Algorithm %d setting max_time to %f seconds\n",
                  (int)index, next->second.to_double ());

                algorithms[index].max_time = next->second.to_double ();
              }
              else
              {
                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_ERROR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Unable to process %s = %s.\n",
                  next->first.c_str (), next->second.to_string ().c_str ());
              } // end unknown arg
            } // end algorithm arg index found
            else // begin non-index prefixed argument
            {
              if (next->first == "repeat")
              {
                repeat = (int)next->second.to_integer ();

                madara_logger_ptr_log (gams::loggers::global_logger.get (),
                  gams::loggers::LOG_ERROR,
                  "gams::algorithms::ExecutorFactory::create:" \
                  " Setting repeat to %d.\n",
                  repeat);
              }
            } // end non-index prefixed argument
          } // end if args string is not empty
        } // end iteration over args

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::ExecutorFactory::create:" \
          " Creating Executor with %d algorithms and %d repeat.\n",
          (int)algorithms.size (), repeat);

        result = new Executor (algorithms, repeat,
          knowledge, platform, sensors, self, agents);
      } // end size > 0
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::ExecutorFactory::create:" \
          " size <= 0. No algorithms to run.\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::ExecutorFactory::create:" \
        " no size found. Must set algorithm.size to the num of algorithms\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::ExecutorFactory::create:" \
      " knowledge, sensors, platform, self, or agents are invalid\n");
  }

  return result;
}

gams::algorithms::Executor::Executor (
  AlgorithmMetaDatas algorithms,
  int repeat,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform, variables::Sensors * sensors,
  variables::Self * self, variables::Agents * agents) :
  BaseAlgorithm (knowledge, platform, sensors, self, agents),
  algorithms_ (algorithms), repeat_ (repeat), alg_index_ (0), cycles_ (0),
  precond_met_ (false), current_ (0)
{
  status_.init_vars (*knowledge, "executor", self->agent.prefix);
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Executor::constr:" \
    " Initialized with %d algorithms and %d repeat.\n",
    (int)algorithms.size (), repeat);
}

gams::algorithms::Executor::~Executor ()
{
}

void
gams::algorithms::Executor::operator= (Executor& rhs)
{
  if (this != &rhs)
  {
    this->algorithms_ = rhs.algorithms_;
    this->repeat_ = rhs.repeat_;
    this->alg_index_ = rhs.alg_index_;
    this->cycles_ = rhs.cycles_;

    this->BaseAlgorithm::operator=(rhs);
  }
}

int
gams::algorithms::Executor::analyze (void)
{
  int result (OK);
  bool create_algorithm (false);

  if (!precond_met_ && status_.finished.is_false ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::Executor::analyze:" \
      " Cycle %d: Precondition for algorithm %d not met yet. Checking...\n",
      cycles_, (int)alg_index_);

    // if there is no precondition, set precond_met to true
    if (algorithms_[alg_index_].precond == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::Executor::analyze:" \
        " Cycle %d: Precondition empty for algorithm %d." \
        " Changing precond_met...\n",
        cycles_, (int)alg_index_);

      precond_met_ = true;

      create_algorithm = true;
    }
    else
    {
      precond_met_ =
        knowledge_->evaluate (algorithms_[alg_index_].precond).is_true ();

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Executor::analyze:" \
        " Cycle %d: Precondition check for algorithm %d is %s.\n",
        cycles_, (int)alg_index_, precond_met_ ? "true" : "false");

      create_algorithm = precond_met_;
    }
  }

  if (create_algorithm)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::analyze:" \
      " Cycle %d: Precondition met for algorithm %d. Creating algorithm\n",
      cycles_, (int)alg_index_);

    current_ = algorithms::global_algorithm_factory->create (
      algorithms_[alg_index_].id,
      algorithms_[alg_index_].args);

    if (algorithms_[alg_index_].max_time > 0)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::Executor::analyze:" \
        " Cycle %d: Maximum time for algorithm %d set to %f\n",
        cycles_, (int)alg_index_, algorithms_[alg_index_].max_time);

      ACE_Time_Value delay;
      delay.set (algorithms_[alg_index_].max_time);
      end_time_ = ACE_OS::gettimeofday () + delay;
    }
  }

  if (precond_met_ && status_.finished.is_false ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::analyze:" \
      " Cycle %d: Analyzing algorithm %d\n",
      cycles_, (int)alg_index_);

    current_->analyze ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::analyze:" \
      " Cycle %d: Algorithm %d precond = %s, finished = %d\n",
      cycles_, (int)alg_index_, precond_met_ ? "true" : "false",
      (int)*status_.finished);
  }

  return result;
}
      
int
gams::algorithms::Executor::execute (void)
{
  int result (OK);

  if (precond_met_ && status_.finished.is_false ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::execute:" \
      " Cycle %d: Calling algorithm %d execute\n",
      cycles_, (int)alg_index_);

    current_->execute ();

    // check if the algorithm status is finished or we've hit end time
    if (current_->get_algorithm_status ()->finished.is_true () ||
      (algorithms_[alg_index_].max_time > 0 &&
         ACE_OS::gettimeofday () > end_time_))
    {
      // if we are at the end of the algorithms list
      if (alg_index_ == algorithms_.size () - 1)
      {
        ++cycles_;

        if (cycles_ >= repeat_ && repeat_ >= 0)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "Executor::execute:" \
            " Cycle %d: Algorithm is finished after %d cycles\n",
            cycles_, cycles_);

          result |= FINISHED;
          status_.finished = 1;
        } // end if finished
        else
        {
          // reset the move index if we are supposed to cycle
          alg_index_ = 0;
          precond_met_ = false;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_MAJOR,
            "Executor::execute:" \
            " Cycle %d: Proceeding to algorithm 0 in next cycle.\n",
            cycles_);

        } // end if not finished
      } // end if move_index_ == end of locations
      else
      {
        // go to the next algorithm
        ++alg_index_;
        precond_met_ = false;

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "Execute::execute:" \
          " Cycle %d: Proceeding to algorithm %d.\n",
          cycles_, (int)alg_index_);

      } // if not the end of the list
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::execute:" \
      " Cycle %d: Algorithm %d precond = %s, finished = %d\n",
      cycles_, (int)alg_index_, precond_met_ ? "true" : "false",
      (int)*status_.finished);
  }

  return result;
}

int
gams::algorithms::Executor::plan (void)
{
  int result (OK);

  if (precond_met_ && status_.finished.is_false ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::plan:" \
      " Cycle %d: Calling algorithm %d plan\n",
      cycles_, (int)alg_index_);

    current_->plan ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Executor::plan:" \
      " Cycle %d: Algorithm %d precond = %s, finished = %d\n",
      cycles_, (int)alg_index_, precond_met_ ? "true" : "false",
      (int)*status_.finished);
  }

  return result;
}
