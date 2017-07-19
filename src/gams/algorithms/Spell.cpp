/**
 * Copyright (c) 2017 Carnegie Mellon University. All Rights Reserved.
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
 *      are those of the author (s) and do not necessarily reflect the views of
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
 * @file Spell.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 **/

#include "gams/algorithms/Spell.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>
#include <ctype.h>
#include <initializer_list>
#include <array>

#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/utility/CartesianFrame.h"
#include "gams/groups/GroupFactoryRepository.h"
#include "madara/utility/Utility.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

using namespace gams::utility;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap   KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::SpellFactory::create (
const madara::knowledge::KnowledgeMap & args,
madara::knowledge::KnowledgeBase * knowledge,
platforms::BasePlatform * platform,
variables::Sensors * sensors,
variables::Self * self,
variables::Agents * agents)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::SpellFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string group = "";
    std::string text = "";
    pose::Pose origin (INVAL_COORD, INVAL_COORD);
    double height = 10;
    double width = 8;
    double buffer = 2;
    std::string barrier_name = "barrier.spell";

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 'b':
        if (i->first == "buffer")
        {
          buffer = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set buffer to %f\n", buffer);
          break;
        }
        else if (i->first == "barrier")
        {
          barrier_name = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set barrier name to %s\n", barrier_name.c_str ());
          break;
        }
        goto unknown;
      case 'g':
        if (i->first == "group")
        {
          group = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set group to %s\n", group.c_str());
          break;
        }
        goto unknown;
      case 'h':
        if (i->first == "height")
        {
          height = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set height to %f\n", height);
          break;
        }
        goto unknown;
      case 'o':
        if (i->first == "origin")
        {
          origin.frame (platform->get_frame ());
          origin.from_container (i->second.to_doubles ());

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set origin to %s\n", origin.to_string().c_str());
          break;
        }
        goto unknown;
      case 't':
        if (i->first == "text")
        {
          text = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set text to %s\n", text.c_str());
          break;
        }
        goto unknown;
      case 'w':
        if (i->first == "width")
        {
          width = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::SpellFactory:" \
            " set width to %f\n", width);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::SpellFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    result = new Spell (
      group, std::move(text), origin,
      height, width, buffer, barrier_name,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Spell::Spell (
  const std::string &group, std::string text,
  pose::Pose origin, double height, double width,
  double buffer,
  const std::string & barrier_name,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self),
  group_factory_ (knowledge),
  group_ (0),
  text_ (std::move(text)), origin_ (origin),
  height_ (height), width_ (width), buffer_ (buffer),
  index_ (-1), step_ (0),
  next_pos_ (INVAL_COORD, INVAL_COORD, INVAL_COORD)
{
  if (knowledge && self)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Tet::constructor:" \
      " Creating algorithm with args: ...\n" \
      "   group -> %s\n" \
      "   text -> %s\n" \
      "   origin -> %s\n" \
      "   barrier_name -> %s\n" \
      "   height -> %f\n" \
      "   width -> %f\n" \
      "   buffer -> %f\n",
      group.c_str (), text_.c_str (), origin.to_string ().c_str (),
      barrier_name.c_str (), height, width, buffer
      );

    status_.init_vars (*knowledge, "spell", self->agent.prefix);
    status_.init_variable_values ();

    // use the group factory to allow for fixed or dynamic groups
    group_ = group_factory_.create (group);

    // fill the group member lists with current contents
    // we can sync the group and call get_members again if we want to support
    // changing group member lists (even with fixed list groups)
    group_->get_members (group_members_);

    // retrieve the index of the agent in the member list
    index_ = get_index ();

    count_ = index_ / 3;
    node_ = index_ % 3;


    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::algorithms::Spell::constructor:" \
      " index: %i\n", index_);

    std::stringstream s;
    s << barrier_name << "." << count_;
    // initialize the barrier with the expected group size. Note that if
    // we want to support dynamic groups, we need to update the barrier
    // with the new group size (if we want to do this in the future)
    barrier_.set_name (s.str(), *knowledge, node_, 3);

    // set the initial barrier to the first position
    barrier_.set (0);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::algorithms::Spell::constructor:" \
      " created barrier: %s with 3 members; I am index %i\n",
        s.str().c_str(), node_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::algorithms::Spell::constructor:" \
      " finished\n");
  }
}


int
gams::algorithms::Spell::get_index () const
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::Spell::get_index:" \
    " looking for: %s\n", self_->agent.prefix.c_str ());

  for (int i = 0; i < group_members_.size (); ++i)
  {
    if (group_members_[i] == self_->agent.prefix)
      return i;
  }

  return -1;
}

int
gams::algorithms::Spell::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Spell::analyze:" \
    " entering analyze method\n");

  return OK;
}

int
gams::algorithms::Spell::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Spell::execute:" \
    " entering execute method\n");

  if (barrier_.is_done () && next_pos_.is_set ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Spell::execute:" \
      " next location for agent is [%s]\n",
      next_pos_.to_string ().c_str ());

    if (platform_->move (next_pos_) == platforms::PLATFORM_ARRIVED) {
      // proceed to next barrier round
      barrier_.next ();
      ++step_; // could also do = (size_t)barrier_.get_round ();
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Spell::execute:" \
      " next location is invalid. Not moving.\n");
  }

  return OK;
}

namespace gams {

class Stroke
{
public:
  typedef std::array<double, 2> Offsets;

  Stroke(std::initializer_list<Offsets> positions)
    : positions_(positions) {}

  pose::Position get_pos (const pose::ReferenceFrame &frame,
                          double height, double width,
                          size_t step) const {
    const Offsets &offsets = positions_[step % positions_.size()];
    return pose::Position(frame, width * offsets[0], height * -offsets[1]);
  }
private:
  std::vector<Offsets> positions_;
};

class Letter
{
public:
  Letter(std::initializer_list<Stroke> strokes)
    : strokes_(strokes) {}

  pose::Position get_pos (const pose::ReferenceFrame &frame,
                          double height, double width,
                          size_t node, size_t step) const {
    if (node >= strokes_.size()) {
      return pose::Position(INVAL_COORD, INVAL_COORD);
    }
    const Stroke &stroke = strokes_[node];
    return stroke.get_pos (frame, height, width, step);
  }
private:
  std::vector<Stroke> strokes_;
};

std::map<char, Letter> create_letters_map ()
{
  return {
    {'A', {
        { {0.5, 0}, {0.25, 0.5}, {0, 1}, {0.25, 0.5} },
        { {0.25, 0.5}, {0.5, 0.5}, {0.75, 0.5}, {0.5, 0.5} },
        { {1, 1}, {0.75, 0.5}, {0.5, 0}, {0.75, 0.5} },
    }},
    {'B', {
      {{0.0, 0.0}, {1.0, 0.25}, {0.0, 0.5}, {1.0, 0.25}},
      {{0.0, 0.5}, {1.0, 0.75}, {0.0, 1.0}, {1.0, 0.75}},
      {{0.0, 1.0}, {0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}},
    }},
    {'C', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}, {0.0, 1.0}},
      {{1.0, 1.0}, {0.5, 1}, {0.0, 1.0}, {0.5, 1}},
    }},
    {'D', {
      {{0, 0}, {0.5, 0.25}, {1, 0.5}, {0.5, 0.25}},
      {{1, 0.5}, {0.5, 0.75}, {0.0, 1.0}, {0.5, 0.75}},
      {{0.0, 1.0}, {0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}},
    }},
    {'E', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}, {0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}, {0.0, 1.0}, {0.0, 0.5}, {0.5, 0.5}, {1.0, 0.5}, {0.5, 0.5}},
      {{1.0, 1.0}, {0.5, 1}, {0.0, 1.0}, {0.5, 1}, {1.0, 1.0}, {0.5, 1}, {0.0, 1.0}, {0.5, 1}},
    }},
    {'F', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{1, 0.5}, {0.5, 0.5}, {0, 0.5}, {0.5, 0.5}},
      {{0.0, 1.0}, {0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}},
    }},
    {'G', {
      {{1.0, 0.0}, {0.15, .15}, {0.0, 0.5}, {0.15, 0.15}},
      {{0.0, 0.5}, {0.15, 0.85}, {1.0, 1.0}, {0.15, 0.85}},
      {{1.0, 1.0}, {1.0, 0.5}, {0.5, 0.5}, {1.0, 0.5}},
    }},
    {'H', {
      {{0, 0}, {0, 0.5}, {0, 1}, {0, 0.5}},
      {{0.0, 0.5}, {0.5, 0.5}, {1.0, 0.5}, {0.5, 0.5}},
      {{1, 1}, {1, 0.5}, {1, 0}, {1, 0.5}},
    }},
    {'I', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.5, 0}, {0.5, 0.5}, {0.5, 1}, {0.5, 0.5}},
      {{1.0, 1.0}, {0.5, 1}, {0.0, 1.0}, {0.5, 1}},
    }},
    {'J', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.5, 0.0}, {0.5, 0.3}, {0.5, 0.6}, {0.5, 0.3}},
      {{0.5, 0.6}, {0.25, 1}, {0.0, 0.6}, {0.25, 1}},
    }},
    {'L', {
      {{0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}, {0.0, 0.0}},
      {{0.0, 1.0}, {0.5, 1.0}, {0.0, 1.0}, {0.0, 0.5}},
      {{0.5, 1.0}, {1.0, 1.0}, {0.5, 1.0}, {1.0, 1.0}},
    }},
    {'M', {
      {{0, 0}, {0, 0.5}, {0, 1}, {0, 0.5}},
      {{0.5, 0.5}, {0, 0}, {0.5, 0.5}, {1, 0}},
      {{1, 1}, {1, 0.5}, {1, 0}, {1, 0.5}},
    }},
    {'N', {
      {{0, 0}, {0, 0.5}, {0, 1}, {0, 0.5}},
      {{0.5, 0.5}, {0, 0}, {0.5, 0.5}, {1, 1}},
      {{1, 1}, {1, 0.5}, {1, 0}, {1, 0.5}},
    }},
    {'O', {
      {{0.5, 0.0}, {0.85, 0.15}, {1, 0.5}, {0.85, 0.85}, {0.5, 1.0}, {0.15, 0.85}, {0.0, 0.5}, {0.15, 0.15}, },
      {{0.15, 0.85}, {0.0, 0.5}, {0.15, 0.15}, {0.5, 0.0}, {0.85, 0.15}, {1, 0.5}, {0.85, 0.85}, {0.5, 1.0}, },
      {{1, 0.5}, {0.85, 0.85}, {0.5, 1.0}, {0.15, 0.85}, {0.0, 0.5}, {0.15, 0.15}, {0.5, 0.0}, {0.85, 0.15}, },
    }},
    {'P', {
      {{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.25}, {0.5, 0.0}},
      {{1.0, 0.25}, {0.5, 0.5}, {0.0, 0.5}, {0.5, 0.5}},
      {{0.0, 1.0}, {0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}},
    }},
    {'Q', {
      {{0.5, 0.0}, {0.85, 0.15}, {1, 0.5}, {0.85, 0.85}, {0.5, 1.0}, {0.15, 0.85}, {0.0, 0.5}, {0.15, 0.15}, },
      {{0.5, 1.0}, {0.15, 0.85}, {0.0, 0.5}, {0.15, 0.15}, {0.5, 0.0}, {0.85, 0.15}, {1, 0.5}, {0.85, 0.85}, },
      {{0.85, 0.85}, {1.0, 1.0}, {0.85, 0.85}, {0.7, 0.7}, {0.85, 0.85}, {1.0, 1.0}, {0.85, 0.85}, {0.7, 0.7}, },
    }},
    {'R', {
      {{0.0, 0.0}, {1.0, 0.25}, {0.0, 0.5}, {1.0, 0.25}},
      {{0.0, 0.5}, {0.5, 0.75}, {1.0, 1.0}, {0.5, 0.75}},
      {{0.0, 1.0}, {0.0, 0.5}, {0.0, 0.0}, {0.0, 0.5}},
    }},
    {'S', {
      {{1.0, 0.2}, {0.5, 0}, {0.0, 0.4}, {0.5, 0}},
      {{0.0, 0.4}, {0.5, 0.5}, {1.0, 0.6}, {0.5, 0.5}},
      {{1.0, 0.6}, {0.5, 1}, {0.0, 0.8}, {0.5, 1.0}},
    }},
    {'T', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.5, 0}, {0.5, 0.5}, {0.5, 0}, {0.5, 0.5}},
      {{0.5, 0.5}, {0.5, 1}, {0.5, 0.5}, {0.5, 1}},
    }},
    {'U', {
      {{0, 0}, {0, 0.5}, {0, 1}, {0, 0.5}},
      {{0.0, 1.0}, {0.5, 1}, {1.0, 1.0}, {0.5, 1}},
      {{1, 1}, {1, 0.5}, {1, 0}, {1, 0.5}},
    }},
    {'V', {
      {{0.0, 0.0}, {0.25, 0.5}, {0.0, 0.0}, {0.25, 0.5}},
      {{0.25, 0.5}, {0.5, 1.0}, {0.75, 0.5}, {0.5, 1.0}},
      {{1.0, 0.0}, {0.75, 0.5}, {1.0, 0.0}, {0.75, 0.5}},
    }},
    {'W', {
      {{0.0, 0.0}, {0.125, 0.5}, {0.25, 1.0}, {0.125, 0.5}},
      {{0.25, 1.0}, {0.5, 0.5}, {0.75, 1.0}, {0.5, 0.5}},
      {{0.875, 0.5}, {0.75, 1.0}, {0.875, 0.5}, {1.0, 0.0}},
    }},
    {'X', {
      {{0.5, 0.5}, {0.25, 0.25}, {0.0, 0.0}, {0.25, 0.25}},
      {{0.0, 1.0}, {0.5, 0.5}, {1.0, 0.0}, {0.5, 0.5}},
      {{1.0, 1.0}, {0.75, 0.75}, {0.5, 0.5}, {0.75, 0.75}},
    }},
    {'Y', {
      {{0.0, 0.0}, {0.25, 0.2}, {0.5, 0.6}, {0.25, 0.2}},
      {{0.75, 0.2}, {0.5, 0.6}, {0.75, 0.2}, {1.0, 0.0}},
      {{0.5, 0.6}, {0.5, 0.8}, {0.5, 1.0}, {0.5, 0.8}},
    }},
    {'Z', {
      {{0, 0}, {0.5, 0}, {1, 0}, {0.5, 0}},
      {{0.5, 0.5}, {0.0, 1.0}, {0.5, 0.5}, {1.0, 0.0}},
      {{1.0, 1.0}, {0.5, 1}, {0.0, 1.0}, {0.5, 1}},
    }},
    {'0', {
      {{0.5, 0}, {1, 0.3}, {1, 0.7}, {0.5, 1}, {0, 0.7}, {0, 0.3}, },
      {{1, 0.7}, {0.5, 1}, {0, 0.7}, {0, 0.3}, {0.5, 0}, {1, 0.3}, },
      {{0, 0.7}, {0, 0.3}, {0.5, 0}, {1, 0.3}, {1, 0.7}, {0.5, 1}, },
    }},
    {'1', {
      {{0.0, 0.3}, {0.25, 0.15}, {0.5, 0.0}, {0.25, 0.15}},
      {{0.5, 0.0}, {0.5, 0.5}, {0.5, 1.0}, {0.5, 0.5}},
      {{0.0, 1.0}, {0.5, 1}, {1.0, 1.0}, {0.5, 1}},
    }},
    {'2', {
      {{0.0, 0.3}, {0.5, 0.0}, {1.0, 0.3}, {0.5, 0.0}},
      {{1.0, 0.3}, {0.5, 0.5}, {0.0, 1.0}, {0.5, 0.5}},
      {{0.0, 1.0}, {0.5, 1}, {1.0, 1.0}, {0.5, 1}},
    }},
    {'3', {
      {{0.0, 0.25}, {0.5, 0.0}, {1.0, 0.25}, {0.5, 0.0}},
      {{1.0, 0.25}, {0.5, 0.5}, {1.0, 0.75}, {0.5, 0.5}},
      {{1.0, 0.75}, {0.5, 1.0}, {0.0, 0.75}, {0.5, 1}},
    }},
    {'4', {
      {{0.0, 0.0}, {0.0, 0.5}, {1.0, 0.5}, {0.0, 0.5}},
      {{1.0, 0.5}, {1.0, 0.25}, {1.0, 0.0}, {1.0, 0.25}},
      {{1.0, 0.75}, {1.0, 0.5}, {1.0, 0.75}, {1.0, 1.0}},
    }},
    {'5', {
      {{1.0, 0.0}, {0.0, 0.0}, {0.0, 0.5}, {0.0, 0.0}},
      {{0.0, 0.5}, {0.5, 0.5}, {1.0, 0.75}, {0.5, 0.5}},
      {{1.0, 0.75}, {0.5, 1.0}, {0.0, 1.0}, {0.5, 1.0}},
    }},
  };
}

const Letter *get_letter (char letter)
{
  static std::map<char, Letter> letters = std::move(create_letters_map ());
  auto search = letters.find(letter);
  if(search == letters.end()) {
    return nullptr;
  } else {
    return &search->second;
  }
}

}

int
gams::algorithms::Spell::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Spell::plan:" \
    " entering plan method\n");

  if (index_ < 0) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Spell::plan:" \
      " invalid index: %i\n", index_);
    return OK;
  }
  if (count_ >= text_.size()) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Spell::plan:" \
      " node %i would go beyond text length: %i\n", count_, text_.size());
    return OK;
  }

  char c = ::toupper(text_[count_]);
  const Letter *letter = get_letter(c);

  if (letter == nullptr) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Spell::plan:" \
      " no pattern set for character: %c\n", c);
    return OK;
  }

  double offset = count_ * (width_ + buffer_);
  pose::CartesianFrame base_frame (origin_);
  pose::CartesianFrame frame (pose::Position(base_frame, offset, 0));

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_WARNING,
    "gams::algorithms::Spell::plan:" \
    " get position for node %i at step %i\n", node_, step_);

  next_pos_ = letter->get_pos(frame, height_, width_, node_, step_)
    .transform_to(platform_->get_frame());

  return OK;
}
