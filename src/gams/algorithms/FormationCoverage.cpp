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
 * @file FormationCoverage.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Definitions for FormationCoverage class
 * - The head agent runs some area coverage algorithm
 * - Followers perform formation flying around the head agent
 **/

#include "gams/algorithms/FormationCoverage.h"
#include "gams/algorithms/AlgorithmFactoryRepository.h"

#include <sstream>
#include <string>
#include <iostream>

#include "madara/utility/Utility.h"

#include "gams/algorithms/AlgorithmFactory.h"

using std::string;
using std::stringstream;
using std::cerr;
using std::endl;

namespace engine = madara::knowledge;
namespace containers = engine::containers;
namespace groups = gams::groups;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap    KnowledgeMap;
typedef groups::AgentVector  AgentVector;

gams::algorithms::BaseAlgorithm *
gams::algorithms::FormationCoverageFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * /*agents*/)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::area_coverage::FormationCoverageFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string head;
    std::vector <double> offset;
    std::vector <double> destination;
    std::string group;
    std::string modifier ("default");
    std::string coverage;
    KnowledgeMap coverage_args;

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage:" \
        " check arg %s\n",
        i->first.c_str ());

      switch (i->first[0])
      {
      case 'c':
        if (i->first == "coverage")
        {
          coverage = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " setting coverage to %s\n", coverage.c_str ());
        }
        else if (madara::utility::begins_with (i->first, "coverage.args.") &&
                 i->first.size () >= 15)
        {
          std::string arg = i->first.substr (14);
          coverage_args[arg] = i->second;

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " adding coverage_arg %s = %s\n", arg.c_str (),
            i->second.to_string ().c_str ());

          break;
        }
        goto unknown;
      case 'd':
        if (i->first == "destination")
        {
          destination = i->second.to_doubles ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " %d size destination set\n", (int)destination.size ());
          break;
        }
        goto unknown;
      case 'g':
        if (i->first == "group")
        {
          group = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " setting group to %s\n", group.c_str ());
          break;
        }
        goto unknown;
      case 'h':
        if (i->first == "head")
        {
          head = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " setting formation head to %s\n", head.c_str ());
          break;
        }
        goto unknown;
      case 'm':
        if (i->first == "modifier")
        {
          modifier = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " setting modifier to %s\n", modifier.c_str ());
          break;
        }
        goto unknown;
      case 'o':
        if (i->first == "offset")
        {
          offset = i->second.to_doubles ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " %d size offset set\n", (int)offset.size ());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "target")
        {
          head = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::FormationCoverage:" \
            " setting formation head/target to %s\n", head.c_str ());
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationCoverage:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());

        break;
      }
    }

    // if group has not been set, use the swarm
    if (head == "" || offset.size () == 0 || destination.size () == 0 ||
        group == "" || coverage == "")
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverage::create:" \
        " Invalid args. head = %s, group = %s, coverage = %s, " \
        " offset.size = %d, destination.size = %d. Returning null.\n",
        head.c_str (), group.c_str (), coverage.c_str (),
        (int)offset.size (), (int)destination.size ());
    }
    else
    {
      result = new FormationCoverage (
        head, offset, destination, group, modifier, coverage, coverage_args,
        knowledge, platform, sensors, self);
    }
  }

  return result;
}

gams::algorithms::FormationCoverage::FormationCoverage (
  const std::string & head_id,
  const std::vector<double> & offset,
  const std::vector<double> & /*destination*/,
  const std::string & group_name,
  const std::string & modifier,
  const std::string & coverage,
  const madara::knowledge::KnowledgeMap & cover_args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) : 
  BaseAlgorithm (knowledge, platform, sensors, self),
  head_algo_ (0),
  is_covering_(false),
  my_formation_ (0)
{
  status_.init_vars (*knowledge, "formation_coverage", self->agent.prefix);
  status_.init_variable_values ();

  // setup formation flying with null destination
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationCoverage::constructor:" \
    " creating formation algorithm\n");

  std::vector<double> dest (3, 0.0);
  my_formation_ = new FormationFlying (head_id, offset, dest, group_name, 
    modifier, knowledge, platform, sensors, self);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::FormationCoverage::constructor:" \
    " successfully created formation algorithm\n");

  if (my_formation_->is_head ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::FormationCoverage::constructor:" \
      " entering leader agent specific code\n");

    BaseAlgorithm * base_algo =
      global_algorithm_factory()->create (coverage, cover_args);
    head_algo_ = dynamic_cast<area_coverage::BaseAreaCoverage*>(base_algo);

    if (head_algo_ == 0)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverage::constructor:" \
        " unable to create area_coverage algorithm \"%s\"\n",
        coverage.c_str ());
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::FormationCoverage::constructor:" \
        " created area_coverage algorithm \"%s\"\n",
        coverage.c_str ());

      // TODO: works for now, but change this to use self_.agents.dest
      stringstream head_destination_str;
      head_destination_str << head_id << ".destination";
      string dest_str = head_destination_str.str ();
      head_destination_.set_name(dest_str, *knowledge, 3);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::FormationCoverage::constructor:" \
      " not leader agent\n");
  }
}

gams::algorithms::FormationCoverage::~FormationCoverage ()
{
  delete my_formation_;
  my_formation_ = 0;
  delete head_algo_;
  head_algo_ = 0;
}

void
gams::algorithms::FormationCoverage::operator= (const FormationCoverage & rhs)
{
  if (this != &rhs)
  {
    //area_coverage::BaseAreaCoverage* head_algo_;
    //bool is_covering_;
    //FormationFlying* my_formation_;
    //madara::knowledge::containers::NativeDoubleArray head_destination_;
    this->BaseAlgorithm::operator= (rhs);
  }
}

int
gams::algorithms::FormationCoverage::analyze (void)
{
  if (my_formation_->is_head ())
  {
    if (is_covering_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage::analyze:" \
        " head coverage analyze\n");
      head_algo_->analyze ();
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage::analyze:" \
        " head formation analyze\n");
      my_formation_->analyze ();
      is_covering_ = my_formation_->is_ready ();
    }
  }
  else // follower
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::FormationCoverage::analyze:" \
      " follower analyze\n");
    my_formation_->analyze ();
  }

  return OK;
}
      
int
gams::algorithms::FormationCoverage::execute (void)
{
  if (my_formation_->is_head ())
  {
    if (is_covering_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage::execute:" \
        " head formation execute\n");
      head_algo_->execute ();
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage::execute:" \
        " head does nothing\n");
    }
  }
  else // follower
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::FormationCoverage::execute:" \
      " follower formation execute\n");
    my_formation_->execute();
  }

  return 0;
}

int
gams::algorithms::FormationCoverage::plan (void)
{
  if (platform_ && *platform_->get_platform_status ()->movement_available)
  {
    if (my_formation_->is_head ())
    {
      if (is_covering_)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::algorithms::FormationCoverage::plan:" \
          " head coverage plan\n");
        head_algo_->plan ();
      }
      head_algo_->get_next_position ().to_container (head_destination_);
    }
    else // follower
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverage::plan:" \
        " follower formation plan\n");
      my_formation_->plan ();
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "FormationCoverage:plan" \
      " platform has not set movement_available to 1.\n");
  }

  return 0;
}
