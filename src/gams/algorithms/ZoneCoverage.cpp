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
 * @file ZoneCoverage.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 **/

#include "gams/algorithms/ZoneCoverage.h"
#include "gams/algorithms/ControllerAlgorithmFactory.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/utility/ArgumentParser.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;

gams::algorithms::BaseAlgorithm *
gams::algorithms::ZoneCoverageFactory::create (
const madara::knowledge::KnowledgeVector & args,
madara::knowledge::KnowledgeBase * knowledge,
platforms::BasePlatform * platform,
variables::Sensors * sensors,
variables::Self * self,
variables::Devices * devices)
{
  BaseAlgorithm * result (0);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverageFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string protectors = "protectors";
    std::string assets = "assets";
    std::string enemies = "enemies";
    std::string formation = "line";
    double buffer = 0.00001;

    utility::ArgumentParser argp(self_->device.command.get_name() + ".",
                                 *knowledge_, args);

    for(utility::ArgumentParser::const_iterator i = argp.begin();
         i != argp.end(); i.next())
    {
      std::string name(i.name());
      if(name.size() <= 0)
        continue;
      switch(name[0])
      {
      case 'a':
        if("assets" == name)
        {
          assets = i.value().to_string();
          break;
        }
        goto unknown;
      case 'b':
        if("buffer" == name)
        {
          buffer = i.value().to_double();
          break;
        }
        goto unknown;
      case 'e':
        if("enemies" == name)
        {
          enemies = i.value().to_string();
          break;
        }
        goto unknown;
      case 'f':
        if("formation" == name)
        {
          formation = i.value().to_string();
          break;
        }
        goto unknown;
      case 'p':
        if("protectors" == name)
        {
          protectors = i.value().to_string();
          break;
        }
        goto unknown;
      default:
      unknown:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::ZoneCoverageFactory:" \
          " argument unknown: %s -> %s\n",
          name.c_str(), i.value().to_string().c_str());
        continue;
      }
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverageFactory:" \
        " argument: %s -> %s\n", name.c_str(), i.value().to_string().c_str());
    }

    result = new ZoneCoverage (
      protectors, assets, enemies, formation, buffer,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::ZoneCoverage::ZoneCoverage (
  const std::string &protectors,
  const std::string &assets,
  const std::string &enemies,
  const std::string &formation,
  double buffer,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  protector_group_(protectors), asset_group_(assets), enemy_group_(enemies),
  protectors_(get_group(protectors)), assets_(get_group(assets)),
  enemies_(get_group(enemies)),
  formation_(formation), buffer_(buffer),
  index_(get_index()),
  form_func_(get_form_func(formation)),
  next_loc_(utility::GPSPosition(DBL_MAX, DBL_MAX, 0)),
  BaseAlgorithm (knowledge, platform, sensors, self)
{
  status_.init_vars (*knowledge, "zone_coverage", self->id.to_integer ());
  status_.init_variable_values ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::constructor:" \
    " Creating algorithm with args: ...\n" \
    "   protectors -> %s\n" \
    "   assets -> %s\n" \
    "   enemies -> %s\n" \
    "   formation -> %s\n" \
    "   buffer -> %f\n",
    protectors.c_str(), assets.c_str(), enemies.c_str(),
    formation.c_str(), buffer
    );

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::ZoneCoverage::constructor:" \
    " protectors list size: %i\n",
    protectors_.size());

  update_arrays(assets_, asset_loc_cont_);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::ZoneCoverage::constructor:" \
    " assets list size: %i; asset loc array size: %i\n",
    assets_.size(), asset_loc_cont_.size());

  update_arrays(enemies_, enemy_loc_cont_);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::ZoneCoverage::constructor:" \
    " enemy list size: %i; enemy loc array size: %i\n",
    assets_.size(), asset_loc_cont_.size());

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::ZoneCoverage::constructor:" \
    " index: %i\n", index_);
}


gams::algorithms::ZoneCoverage::~ZoneCoverage ()
{
}

void
gams::algorithms::ZoneCoverage::operator= (const ZoneCoverage & rhs)
{
  if (this != &rhs)
  {
    this->BaseAlgorithm::operator= (rhs);
    this->protectors_ = rhs.protectors_;
    this->assets_ = rhs.assets_;
    this->enemies_ = rhs.enemies_;
    this->formation_ = rhs.formation_;
    this->buffer_ = rhs.buffer_;
    this->form_func_ = rhs.form_func_;
  }
}

int
gams::algorithms::ZoneCoverage::get_index() const
{
  std::stringstream to_find;
  to_find << "device." << self_->id.to_integer();
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::algorithms::ZoneCoverage::get_index:" \
    " looking for: %s\n", to_find.str().c_str());
  for(int i = 0; i < protectors_.size(); ++i)
  {
    if(protectors_[i] == to_find.str())
      return i;
  }
  return -1;
}

madara::knowledge::containers::StringVector
gams::algorithms::ZoneCoverage::get_group(const std::string &name) const
{
  return madara::knowledge::containers::StringVector(
     "group." + name, *knowledge_);
}

void
gams::algorithms::ZoneCoverage::update_arrays(
  const madara::knowledge::containers::StringVector &names,
  MadaraArrayVec &arrays) const
{
  arrays.clear();
  arrays.resize(names.size());
  for(int i = 0; i < names.size(); ++i)
  {
    arrays[i].set_name(names[i] + ".location", *knowledge_, 3);
  }
}

void
gams::algorithms::ZoneCoverage::update_locs(
  const MadaraArrayVec &arrays,
  std::vector<utility::GPSPosition> &locs) const
{
  if(locs.size() != arrays.size())
    locs.resize(arrays.size());
  for(int i = 0; i < arrays.size(); ++i)
  {
    locs[i].from_container(
     const_cast<madara::knowledge::containers::NativeDoubleArray &>(arrays[i]));
  }
}

int
gams::algorithms::ZoneCoverage::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::analyze:" \
    " entering analyze method\n");

  update_locs(asset_loc_cont_, asset_locs_);
  update_locs(enemy_loc_cont_, enemy_locs_);
  return OK;
}

int
gams::algorithms::ZoneCoverage::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::execute:" \
    " entering execute method\n");

  if(next_loc_.latitude() != DBL_MAX)
    platform_->move(next_loc_);
  return OK;
}

int
gams::algorithms::ZoneCoverage::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::plan:" \
    " entering plan method\n");

  next_loc_ = ((this)->*(form_func_))();
  return OK;
}

gams::utility::GPSPosition
gams::algorithms::ZoneCoverage::line_formation() const
{
  utility::GPSPosition ret(DBL_MAX, DBL_MAX, 0);

  if(asset_locs_.size() >= 1 && enemy_locs_.size() >= 1)
  {
    const utility::GPSPosition &asset_loc = asset_locs_[0];
    const utility::GPSPosition &enemy_loc = enemy_locs_[0];

    utility::GPSPosition middle((asset_loc.x + enemy_loc.x) / 2,
                                (asset_loc.y + enemy_loc.y) / 2,
                                (asset_loc.z + enemy_loc.z) / 2);
    if(index_ == 0)
      ret = middle;
    else
    {
      int offset = (index_ % 2 == 0) ? (index_ / 2) : (- (index_ + 1) / 2);
      double a = atan2(enemy_loc.x - asset_loc.x, enemy_loc.y - asset_loc.y);
      double pa = a + M_PI / 2;
      double xo = sin(pa);
      double yo = cos(pa);
      ret.x = middle.x + xo * offset * buffer_;
      ret.y = middle.y + yo * offset * buffer_;
      ret.z = middle.z;
    }
  }

  return ret;
}

gams::algorithms::ZoneCoverage::formation_func
gams::algorithms::ZoneCoverage::get_form_func(const std::string &form_name)
{
  return &ZoneCoverage::line_formation;
}
