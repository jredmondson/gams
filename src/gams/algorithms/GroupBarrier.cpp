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
 * @file GroupBarrier.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 **/

#include "ace/High_Res_Timer.h"
#include "ace/OS_NS_sys_time.h"
#include "gams/algorithms/GroupBarrier.h"
#include "gams/algorithms/ControllerAlgorithmFactory.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "gams/algorithms/AlgorithmFactory.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;

gams::algorithms::BaseAlgorithm *
gams::algorithms::GroupBarrierFactory::create (
const madara::knowledge::KnowledgeVector & args,
madara::knowledge::KnowledgeBase * knowledge,
platforms::BasePlatform * platform,
variables::Sensors * sensors,
variables::Self * self,
variables::Agents * agents)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::GroupBarrierFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::vector <std::string> members;

    std::string group = "";
    std::string barrier = "barrier.group_barrier";
    double interval = 1.0;

    for (size_t i = 0; i < args.size (); ++i)
    {
      if (args[i] == "group" && i + 1 < args.size ())
      {
        group = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::GroupBarrierFactory:" \
          " setting group to %s\n", group.c_str ());

        std::string members_list_name = "group." + group + ".members";

        containers::StringVector member_list (members_list_name, *knowledge);

        member_list.copy_to (members);

        ++i;
      }
      else if (args[i] == "interval" && i + 1 < args.size ())
      {
        interval = args[i + 1].to_double ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::GroupBarrierFactory:" \
          " setting interval to %.2f\n", interval);

        ++i;
      }
      else if (args[i] == "barrier" && i + 1 < args.size ())
      {
        barrier = args[i + 1].to_string ();

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::GroupBarrierFactory:" \
          " setting barrier to %s\n", barrier.c_str ());

        ++i;
      }
    }

    // if group has not been set, use the swarm
    if (group == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::GroupBarrier::constructor:" \
        " No group specified. Using swarm.\n");

      Integer processes = (Integer)agents->size ();

      for (Integer i = 0; i < processes; ++i)
      {
        madara::knowledge::KnowledgeRecord temp ("agent.");
        temp += i;
        members.push_back (temp.to_string ());
      }
    }

    result = new GroupBarrier (members, barrier, interval,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::GroupBarrier::GroupBarrier (
  const std::vector<std::string> & members,
  std::string barrier_name,
  double interval,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self),
  members_ (members)
{
  status_.init_vars (*knowledge, "barrier", self->id.to_integer ());
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::GroupBarrier::constructor:" \
    " Creating algorithm with args: " \
    " barrier=%s\n",
    barrier_name.c_str ());

  madara::knowledge::KnowledgeRecord temp ("agent.");
  temp += self_->id.to_string ();

  position_ = this->get_position_in_member_list (temp.to_string (), members_);

  interval_.set (interval);

  next_barrier_ = ACE_OS::gettimeofday ();
  next_barrier_ += interval_;

  if (position_ >= 0)
  {
    barrier_.set_name (barrier_name, *knowledge, position_, (int)members.size ());
    barrier_.set (0);
    barrier_.next ();
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::GroupBarrier::constructor:" \
      " agent.%d does not have a position in group algorithm." \
      " Unable to participate in barrier.\n",
      (int)self_->id.to_integer ());
  }
}

int
gams::algorithms::GroupBarrier::get_position_in_member_list (
std::string id,
std::vector <std::string> & member_list)
{
  int result = -1;

  for (size_t i = 0; i < member_list.size (); ++i)
  {
    if (member_list[i] == id)
    {
      result = (int)i;
      break;
    }
  }

  return result;
}

gams::algorithms::GroupBarrier::~GroupBarrier ()
{
}

void
gams::algorithms::GroupBarrier::operator= (const GroupBarrier & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator= (rhs);

    members_ = rhs.members_;
    position_ = rhs.position_;
    barrier_ = rhs.barrier_;
  }
}

int
gams::algorithms::GroupBarrier::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::GroupBarrier::analyze:" \
    " entering analyze method\n");

  if (position_ >= 0)
  {
    int round = barrier_.get_round ();

    barrier_.modify ();

    if (barrier_.is_done ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::GroupBarrier::analyze:" \
        " %d: Round %d: Proceeding to next barrier round\n",
        position_, round);

      ACE_Time_Value current = ACE_OS::gettimeofday ();

      if (current > next_barrier_)
      {
        barrier_.next ();

        while (current > next_barrier_)
        {
          next_barrier_ += interval_;
        }
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
        "gams::algorithms::GroupBarrier::analyze:" \
        " %d: Round %d: NOT proceeding to next barrier round\n",
        position_, round);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
      "gams::algorithms::GroupBarrier::analyze:" \
      " agent.%d does not have a position in group algorithm." \
      " Nothing to analyze.\n",
      (int)self_->id.to_integer ());
  }

  return OK;
}

int
gams::algorithms::GroupBarrier::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::GroupBarrier::execute:" \
    " entering execute method\n");

  return 0;
}

int
gams::algorithms::GroupBarrier::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::GroupBarrier::plan:" \
    " entering plan method\n");

  return 0;
}
