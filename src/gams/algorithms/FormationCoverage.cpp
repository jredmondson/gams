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
#include "gams/algorithms/ControllerAlgorithmFactory.h"

#include <sstream>
#include <string>
#include <iostream>

#include "gams/algorithms/AlgorithmFactory.h"

using std::string;
using std::stringstream;
using std::cerr;
using std::endl;

gams::algorithms::BaseAlgorithm *
gams::algorithms::FormationCoverageFactory::create (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Devices * /*devices*/)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::algorithms::FormationCoverageFactory:" \
    " entered create with %u args\n", args.size ());
  
  if (knowledge && sensors && platform && self && args.size () >= 6)
  {
    // store error value
    bool error = false;

    // create arg types
    madara::knowledge::KnowledgeRecord::Integer target;
    std::vector<double> offset;
    std::vector<madara::knowledge::KnowledgeRecord::Integer> members;
    std::string modifier ("default");
    std::string coverage;

    if (args[0].is_integer_type ())
    {
      target = args[0].to_integer ();
    }
    else
    {
      error = true;
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverageFactory:" \
        " invalid target argument\n");
    }

    if (args[1].is_double_type () || args[1].is_integer_type ())
    {
      offset = args[1].to_doubles ();
      if (offset.size () < 2 || offset.size () > 3)
      {
        error = true;
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::FormationCoverageFactory:" \
          " invalid offset size (expected 2 or 3, got %u)\n", offset.size ());
      }
    }
    else
    {
      error = true;
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverageFactory:" \
        " invalid offset argument type (expected doubles)\n");
    }

    if (args[2].is_integer_type ())
    {
      members = args[2].to_integers ();
      if (members.size () == 0)
      {
        error = true;
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::FormationCoverageFactory:" \
          " invalid members argument size (did not receive any)\n");
      }
    }
    else
    {
      error = true;
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverageFactory:" \
        " invalid members argument type (expected integers)\n");
    }

    if (args[3].is_string_type ())
      modifier = args[3].to_string ();


    if (args[4].is_string_type ())
    {
      coverage = args[4].to_string ();
    }
    else
    {
      error = true;
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::FormationCoverageFactory:" \
        " invalid coverage argument type (expected string)\n");
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
     gams::loggers::LOG_DETAILED,
     "gams::algorithms::FormationCoverageFactory:" \
     " coverage arg is %s\n", coverage.c_str ());

    madara::knowledge::KnowledgeVector cover_args;
    for(size_t i = 5; i < args.size(); ++i)
    {
      cover_args.push_back (args[i]);
      
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::FormationCoverageFactory:" \
        " coverage arg %u is %s\n", i - 5, args[i].to_string ().c_str ());
    }

    result = new FormationCoverage (
      target, offset, members, modifier,
      coverage, cover_args,
      knowledge, platform, sensors, self);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::FormationCoverageFactory:" \
      " invalid knowledge, sensors, platform, self, or arg count\n");
  }

  return result;
}

gams::algorithms::FormationCoverage::FormationCoverage (
  const madara::knowledge::KnowledgeRecord::Integer & head_id,
  const std::vector<double> & offset,
  const std::vector<madara::knowledge::KnowledgeRecord::Integer> & members,
  const std::string & modifier,
  const std::string & coverage,
  const madara::knowledge::KnowledgeVector & cover_args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) : 
  BaseAlgorithm (knowledge, platform, sensors, self), is_covering_(false), 
  head_algo_ (0), my_formation_ (0)
{
  status_.init_vars (*knowledge, "formation_coverage", self->id.to_integer ());
  status_.init_variable_values ();

  // setup formation flying with null destination
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::FormationCoverage::constructor:" \
    " creating formation algorithm\n");

  std::vector<double> dest (3, 0.0);
  my_formation_ = new FormationFlying (head_id, offset, dest, members, 
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

    ControllerAlgorithmFactory factory (knowledge, sensors, platform, self);
    BaseAlgorithm * base_algo = factory.create (coverage, cover_args);
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

      // TODO: works for now, but change this to use self_.devices.dest
      stringstream head_destination_str;
      head_destination_str << "device." << self->id.to_integer () << ".destination";
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

  return 0;
}
