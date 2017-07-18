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
 * @file Text.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 **/

#include "gams/algorithms/Text.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>
#include <initializer_list>
#include <array>

#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/utility/CartesianFrame.h"
#include "madara/utility/Utility.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

using namespace gams::utility;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap   KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::TextFactory::create (
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
    "gams::algorithms::TextFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string group = "";
    std::string text = "";
    pose::Pose origin (INVAL_COORD, INVAL_COORD);
    double height = 10;
    double width = 8;
    double buffer = 2;

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
            "gams::algorithms::TextFactory:" \
            " set buffer to %f\n", buffer);
          break;
        }
        goto unknown;
      case 'g':
        if (i->first == "group")
        {
          group = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::TextFactory:" \
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
            "gams::algorithms::TextFactory:" \
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
            "gams::algorithms::TextFactory:" \
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
            "gams::algorithms::TextFactory:" \
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
            "gams::algorithms::TextFactory:" \
            " set width to %f\n", width);
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::TextFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    result = new Text (
      std::move(group), std::move(text), origin,
      height, width, buffer,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::Text::Text (
  std::string group, std::string text,
  pose::Pose origin, double height, double width,
  double buffer,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self),
  group_ (std::move(group)), text_ (std::move(text)), origin_ (origin),
  height_ (height), width_ (width), buffer_ (buffer),
  nodes_ (get_group (group_)), index_ (get_index ()), step_ (0),
  next_pos_ (INVAL_COORD, INVAL_COORD, INVAL_COORD)
{
  status_.init_vars (*knowledge, "text", self->agent.prefix);
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Tet::constructor:" \
    " Creating algorithm with args: ...\n" \
    "   group -> %s\n" \
    "   text -> %s\n" \
    "   origin -> %s\n" \
    "   height -> %f\n" \
    "   width -> %f\n" \
    "   buffer -> %f\n",
    group_.c_str (), text_.c_str (), origin.to_string().c_str (),
    height, width, buffer
    );

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::Text::constructor:" \
    " index: %i\n", index_);
}


int
gams::algorithms::Text::get_index () const
{
  std::stringstream to_find;
  to_find << "agent." << self_->id.to_integer ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::Text::get_index:" \
    " looking for: %s\n", to_find.str ().c_str ());

  for (int i = 0; i < nodes_.size (); ++i)
  {
    if (nodes_[i] == to_find.str ())
      return i;
  }

  return -1;
}

madara::knowledge::containers::StringVector
gams::algorithms::Text::get_group (const std::string &name) const
{
  return madara::knowledge::containers::StringVector (
     "group." + name + ".members", *knowledge_);
}

int
gams::algorithms::Text::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Text::analyze:" \
    " entering analyze method\n");

  return OK;
}

int
gams::algorithms::Text::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Text::execute:" \
    " entering execute method\n");

  if (next_pos_.is_set ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Text::execute:" \
      " next location for agent is [%s]\n",
      next_pos_.to_string ().c_str ());

    if (platform_->move (next_pos_) == platforms::PLATFORM_ARRIVED) {
      ++step_;
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::Text::execute:" \
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
    return pose::Position(frame, width * offsets[0], height * offsets[1]);
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
      } },
    {'N', {
        { {0, 0}, {0, 0.5}, {0, 1}, {0, 0.5} },
        { {0.5, 0.5}, {0, 0}, {0.5, 0.5}, {1, 1} },
        { {1, 1}, {1, 0.5}, { 1, 0}, {1, 0.5} },
      } },
    {'O', {
        { {0.5, 0}, {1, 0.3}, {1, 0.7}, {0.5, 1}, {0, 0.7}, {0, 0.3}, },
        { {1, 0.7}, {0.5, 1}, {0, 0.7}, {0, 0.3}, {0.5, 0}, {1, 0.3}, },
        { {0, 0.7}, {0, 0.3}, {0.5, 0}, {1, 0.3}, {1, 0.7}, {0.5, 1}, },
      } },
    {'T', {
        { {0, 0}, {0.5, 0}, {1, 0}, {0.5, 0} },
        { {0.5, 0}, {0.5, 0.5}, {0.5, 0}, {0.5, 0.5} },
        { {0.5, 0.5}, {0.5, 1}, {0.5, 0.5}, {0.5, 1} },
      } },
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
gams::algorithms::Text::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::Text::plan:" \
    " entering plan method\n");

  if (index_ < 0) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Text::plan:" \
      " invalid index: %i\n", index_);
    return OK;
  }
  size_t count = index_ / 3;
  size_t node = index_ * 3;

  if (count >= text_.size()) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Text::plan:" \
      " node %i would go beyond text length: %i\n", count, text_.size());
    return OK;
  }

  char c = std::toupper(text_[count]);
  const Letter *letter = get_letter(c);

  if (letter == nullptr) {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_WARNING,
      "gams::algorithms::Text::plan:" \
      " no pattern set for character: %c\n", c);
    return OK;
  }

  double offset = count * (width_ + buffer_);
  pose::CartesianFrame base_frame (origin_);
  pose::CartesianFrame frame (pose::Position(base_frame, offset, 0));

  next_pos_ = letter->get_pos(frame, height_, width_, node, step_);

  return OK;
}
