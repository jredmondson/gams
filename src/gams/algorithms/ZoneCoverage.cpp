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
 * @file ZoneCoverage.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 **/

#include "gams/algorithms/ZoneCoverage.h"
#include "madara/knowledge/containers/StringVector.h"
#include "madara/utility/Utility.h"

#include <sstream>
#include <string>
#include <iostream>
#include <cmath>

#include "gams/algorithms/AlgorithmFactory.h"
#include "gams/utility/CartesianFrame.h"
#include "madara/utility/Utility.h"

namespace engine = madara::knowledge;
namespace containers = engine::containers;

using namespace gams::utility;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;
typedef madara::knowledge::KnowledgeMap   KnowledgeMap;

gams::algorithms::BaseAlgorithm *
gams::algorithms::ZoneCoverageFactory::create (
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
    "gams::algorithms::ZoneCoverageFactory:" \
    " entered create with %u args\n", args.size ());

  if (knowledge && sensors && platform && self)
  {
    std::string protectors = "group.protectors";
    std::string assets = "group.assets";
    std::string enemies = "group.enemies";
    std::string formation = "line";
    double buffer = 2;
    double distance = 0.5;

    for (KnowledgeMap::const_iterator i = args.begin (); i != args.end (); ++i)
    {
      if (i->first.size () <= 0)
        continue;

      switch (i->first[0])
      {
      case 'a':
        if (i->first == "assets")
        {
          assets = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set assets group to %s\n", assets.c_str ());
          break;
        }
        goto unknown;
      case 'b':
        if (i->first == "buffer")
        {
          buffer = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set buffer to %f\n", buffer);
          break;
        }
        goto unknown;
      case 'd':
        if (i->first == "distance")
        {
          distance = i->second.to_double ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set distance to %f\n", distance);
          break;
        }
        goto unknown;
      case 'e':
        if (i->first == "enemies")
        {
          enemies = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set enemies group to %s\n", enemies.c_str ());
          break;
        }
        goto unknown;
      case 'f':
        if (i->first == "formation")
        {
          formation = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set formation to %s\n", formation.c_str ());
          break;
        }
        goto unknown;
      case 'p':
        if (i->first == "protectors")
        {
          protectors = i->second.to_string ();

          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::algorithms::ZoneCoverageFactory:" \
            " set protectors group to %s\n", protectors.c_str ());
          break;
        }
        goto unknown;
      unknown:
      default:
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::FormationFlyingFactory:" \
          " argument unknown: %s -> %s\n",
          i->first.c_str (), i->second.to_string ().c_str ());
        break;
      }
    }

    result = new ZoneCoverage (
      protectors, assets, enemies, formation, buffer, distance,
      knowledge, platform, sensors, self);
  }

  return result;
}

gams::algorithms::ZoneCoverage::ZoneCoverage (
  const std::string &protectors,
  const std::string &assets,
  const std::string &enemies,
  const std::string &formation,
  double buffer, double distance,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self) :
  BaseAlgorithm (knowledge, platform, sensors, self),
  group_factory_ (knowledge),
  formation_ (formation), buffer_ (buffer), distance_ (distance),
  index_ (-1),
  form_func_ (get_form_func (formation)),
  next_loc_ (INVAL_COORD, INVAL_COORD, INVAL_COORD)
{
  if (knowledge)
  {
    status_.init_vars (*knowledge, "zone_coverage", self->agent.prefix);
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
      protectors.c_str (), assets.c_str (), enemies.c_str (),
      formation.c_str (), buffer
      );

    // check if protectors is a single agent or a group
    if (!gams::variables::Agent::is_agent (*knowledge, protectors))
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " protectors is a group prefix: %s\n",
        protectors.c_str ());

      std::string protectors_name;

      // add group. to the group name if it is not provided
      if (madara::utility::begins_with (protectors, "group."))
      {
        protectors_name = protectors;
      }
      else
      {
        protectors_name = "group.";
        protectors_name += protectors;
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " protectors group name in knowledge base is : %s\n",
        protectors_name.c_str ());


      // use the group factory to allow for fixed or dynamic groups
      protectors_ = group_factory_.create (protectors_name);

      if (protectors_)
      {
        // fill the group member lists with current contents
        // we can sync the group and call get_members again if we want to support
        // changing group member lists (even with fixed list groups)
        protectors_->get_members (protectors_members_);
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::ZoneCoverage::constructor:" \
          " protectors group was a null group (group not found)\n");

        knowledge->print ();
      }

      // retrieve the index of the agent in the member list
      index_ = gams::groups::find_member_index (
        self_->agent.prefix, protectors_members_);

      if (index_ < 0)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::ZoneCoverage::constructor:" \
          " this agent (%s) is not in the protectors group (%d members)\n",
          self_->agent.prefix.c_str (), (int)protectors_members_.size ());
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " protectors is an agent prefix: %s\n",
        protectors.c_str ());

      // we have a single agent protector, not a group
      protectors_ = 0;
      protectors_members_.push_back (protectors);
      if (self_->agent.prefix == protectors)
      {
        index_ = 0;
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::ZoneCoverage::constructor:" \
          " this agent (%s) is not in the protectors group\n",
          self_->agent.prefix.c_str ());
      }
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::constructor:" \
      " protectors list size: %i\n",
      protectors_members_.size ());

    // check if assets is a single agent or a group
    if (!gams::variables::Agent::is_agent (*knowledge, assets))
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " assets is a group prefix: %s\n",
        assets.c_str ());

      std::string assets_name;

      // add group. to the group name if it is not provided
      if (madara::utility::begins_with (assets, "group."))
      {
        assets_name = assets;
      }
      else
      {
        assets_name = "group.";
        assets_name += assets;
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " protectors group name in knowledge base is : %s\n",
        assets_name.c_str ());

      // use the group factory to allow for fixed or dynamic groups
      assets_ = group_factory_.create (assets_name);

      if (assets_)
      {
        // fill the group member lists with current contents
        // we can sync the group and call get_members again if we want to support
        // changing group member lists (even with fixed list groups)
        assets_->get_members (assets_members_);
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::ZoneCoverage::constructor:" \
          " assets group was a null group (group not found)\n");

        knowledge->print ();
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " assets is an agent prefix: %s\n",
        assets.c_str ());

      // we have a single agent asset, not a group
      assets_ = 0;
      assets_members_.push_back (assets);
    }

    update_arrays (assets_members_, asset_loc_cont_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::constructor:" \
      " assets list size: %i; asset loc array size: %i\n",
      assets_members_.size (), asset_loc_cont_.size ());

    // check if enemies is a single agent or a group
    if (!gams::variables::Agent::is_agent (*knowledge, enemies))
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " enemies is a group prefix: %s\n",
        enemies.c_str ());

      std::string enemies_name;

      // add group. to the group name if it is not provided
      if (madara::utility::begins_with (enemies, "group."))
      {
        enemies_name = enemies;
      }
      else
      {
        enemies_name = "group.";
        enemies_name += enemies;
      }

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " protectors group name in knowledge base is : %s\n",
        enemies_name.c_str ());

      // use the group factory to allow for fixed or dynamic groups
      enemies_ = group_factory_.create (enemies_name);

      if (enemies_)
      {
        // fill the group member lists with current contents
        // we can sync the group and call get_members again if we want to support
        // changing group member lists (even with fixed list groups)
        enemies_->get_members (enemies_members_);
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::algorithms::ZoneCoverage::constructor:" \
          " enemies group was a null group (group not found)\n");

        knowledge->print ();
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::constructor:" \
        " enemies is an agent prefix: %s\n",
        enemies.c_str ());

      // we have a single agent asset, not a group
      enemies_ = 0;
      enemies_members_.push_back (enemies);
    }

    update_arrays (enemies_members_, enemy_loc_cont_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::constructor:" \
      " enemy list size: %i; enemy loc array size: %i\n",
      enemies_members_.size (), enemy_loc_cont_.size ());

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::constructor:" \
      " index: %i\n", index_);
  }
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

void
gams::algorithms::ZoneCoverage::update_arrays (
  const gams::groups::AgentVector &names,
  MadaraArrayVec &arrays) const
{
  arrays.clear ();

  arrays.resize (names.size ());
  for (int i = 0; i < names.size (); ++i)
  {
    arrays[i].set_name (names[i] + ".location", *knowledge_, 3);
  }
}

void
gams::algorithms::ZoneCoverage::update_locs (
  const MadaraArrayVec &arrays,
  std::vector<Location> &locs) const
{
  if (locs.size () != arrays.size ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::update_locs:" \
      " resizing locs array\n");
    locs.resize (arrays.size (), Location (platform_->get_frame ()));
  }
  for (int i = 0; i < arrays.size (); ++i)
  {
    if (arrays[i].size () >= 2)
    {
      locs[i].from_container (arrays[i]);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::update_locs:" \
        " read loc (%f, %f, %f) for #%i\n",
        locs[i].x (), locs[i].y (), locs[i].z (), i);
    }
  }
}

int
gams::algorithms::ZoneCoverage::analyze (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::analyze:" \
    " entering analyze method\n");

  update_locs (asset_loc_cont_, asset_locs_);
  update_locs (enemy_loc_cont_, enemy_locs_);
  return OK;
}

int
gams::algorithms::ZoneCoverage::execute (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::execute:" \
    " entering execute method\n");

  if (next_loc_.is_set ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::execute:" \
      " next location for agent is [%s]\n",
      next_loc_.to_string ().c_str ());

    platform_->move (next_loc_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::execute:" \
      " next location is invalid. Not moving.\n");
  }



  return OK;
}

int
gams::algorithms::ZoneCoverage::plan (void)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::algorithms::ZoneCoverage::plan:" \
    " entering plan method\n");

  if (index_ >= 0)
    next_loc_ = ( (this)->* (form_func_)) ();

  if (asset_locs_.size () > 0 && enemy_locs_.size () > 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::plan:" \
      " vip is at [%s]. attacker is at [%s]\n",
      asset_locs_[0].to_string ().c_str (),
      enemy_locs_[0].to_string ().c_str ());
  }

  return OK;
}

Location
gams::algorithms::ZoneCoverage::line_formation () const
{
  Location ret (platform_->get_frame ());

  if (asset_locs_.size () >= 1 && enemy_locs_.size () >= 1)
  {
    const Location &asset_loc = asset_locs_[0];
    const Location &enemy_loc = enemy_locs_[0];

    if (asset_loc.is_set () && enemy_loc.is_set ())
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::plan:" \
        " vip is set. attacker is set\n");

      Location middle (platform_->get_frame (),
              (asset_loc.x () * distance_) + (enemy_loc.x () * (1 - distance_)),
              (asset_loc.y () * distance_) + (enemy_loc.y () * (1 - distance_)),
              (asset_loc.z () * distance_) + (enemy_loc.z () * (1 - distance_)));


      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::plan:" \
        " middle location is [%s]\n",
        middle.to_string ().c_str ());


      if (index_ == 0)
      {

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::ZoneCoverage::plan:" \
          " defender is the middle agent. Using middle.\n");

        ret = middle;
      }
      else
      {
        CartesianFrame frame (asset_loc);

        ret.frame (frame);

        Location middle_cart (frame, middle);
        Location enemy_loc_cart (frame, enemy_loc);

        int offset = (index_ % 2 == 0) ? (index_ / 2) : (- (index_ + 1) / 2);
        double a = atan2 (enemy_loc_cart.x (),
                         enemy_loc_cart.y ());
        double pa = a + M_PI / 2;
        double xo = sin (pa);
        double yo = cos (pa);
        ret.x (middle_cart.x () + xo * offset * buffer_);
        ret.y (middle_cart.y () + yo * offset * buffer_);
        ret.z (0);

        ret.transform_this_to (platform_->get_frame ());
        ret.z (middle.z ());


        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::algorithms::ZoneCoverage::plan:" \
          " defender is NOT the middle agent. Using [%s].\n",
          ret.to_string ().c_str ());

      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::ZoneCoverage::plan:" \
        " vip is not set or attacker is not set\n");
    }
  }
  else
  {

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::ZoneCoverage::plan:" \
      " vip locations or enemy locations are not set. " \
      " vip location size is %d, enemy location size is %d\n",
      (int)asset_locs_.size (), (int)enemy_locs_.size ());

  }

  return ret;
}

Location
gams::algorithms::ZoneCoverage::arc_formation () const
{
  Location ret (platform_->get_frame ());

  if (asset_locs_.size () >= 1 && enemy_locs_.size () >= 1)
  {
    const Location &asset_loc = asset_locs_[0];
    const Location &enemy_loc = enemy_locs_[0];

    if (asset_loc.is_set () && enemy_loc.is_set ())
    {
      Location middle (platform_->get_frame (),
              (asset_loc.x () * distance_) + (enemy_loc.x () * (1 - distance_)),
              (asset_loc.y () * distance_) + (enemy_loc.y () * (1 - distance_)),
              (asset_loc.z () * distance_) + (enemy_loc.z () * (1 - distance_)));
      if (index_ == 0)
        ret = middle;
      else
      {
        CartesianFrame frame (asset_loc);

        ret.frame (frame);

        Location enemy_cart_loc (frame, enemy_loc);

        double distance = asset_loc.distance_to (middle);
        double circ = distance * M_PI * 2;

        int offset = (index_ % 2 == 0) ? (index_ / 2) : (- (index_ + 1) / 2);
        double arc_len = offset * buffer_;

        double a = atan2 (enemy_cart_loc.x (), enemy_cart_loc.y ());
        double ao = a + ( ( (arc_len) / circ) * M_PI * 2);
        double xo = sin (ao);
        double yo = cos (ao);
        ret.x (xo * distance);
        ret.y (yo * distance);
        ret.z (0);

        ret.transform_this_to (platform_->get_frame ());
        ret.z (middle.z ());
      }
    }
  }

  return ret;
}

namespace onion
{
  struct placement
  {
    int rank;
    int offset;
  };

  void init_placements (std::vector<placement> &placements, int index)
  {
    if (index >= placements.size ())
    {
      std::vector<int> counts;
      int i = 0;
      size_t last_rank = 0;
      counts.resize (1);
      for (; i < placements.size (); ++i)
      {
        placement cur = placements[i];
        if (cur.rank >= counts.size ())
          counts.resize (cur.rank + 1);
        ++counts[cur.rank];
        last_rank = cur.rank;
      }
      placements.resize (index + 1);
      for (; i < placements.size (); ++i)
      {
        bool want_even = (last_rank == 0 ? false :
                          ( ( ( (last_rank - 1) / 2) % 2) == 0 ? true : false));
        bool is_even = ( (counts[last_rank] % 2) == 0) ? true : false;

        int min_rank_count = is_even ? 2 : 3;

        if (counts[last_rank] < min_rank_count || want_even != is_even)
        {
          /* do nothing */
        }
        else
        {
          ++last_rank;
          if (last_rank == counts.size ())
          {
            if (last_rank >= 2 && counts[last_rank - 1] <= 3)
              last_rank = 0;
            else
              counts.push_back (0);
          }
        }
        placement p = { (int)last_rank, counts[last_rank]};
        placements[i] = p;
        ++counts[last_rank];
        //cout << i << ": " << placements[i].rank << " " << placements[i].offset << endl;
      }
    }
  }

  placement get_placement (int index)
  {
    static std::vector<placement> placements;
    init_placements (placements, index);
    return placements[index];
  }
}

Location
gams::algorithms::ZoneCoverage::onion_formation () const
{
  Location ret (platform_->get_frame ());

  if (asset_locs_.size () >= 1 && enemy_locs_.size () >= 1)
  {
    const Location &asset_loc = asset_locs_[0];
    const Location &enemy_loc = enemy_locs_[0];

    if (asset_loc.is_set () && enemy_loc.is_set ())
    {
      Location middle (platform_->get_frame (),
              (asset_loc.x () * distance_) + (enemy_loc.x () * (1 - distance_)),
              (asset_loc.y () * distance_) + (enemy_loc.y () * (1 - distance_)),
              (asset_loc.z () * distance_) + (enemy_loc.z () * (1 - distance_)));
      if (index_ == 0)
        ret = middle;
      else
      {
        onion::placement p = onion::get_placement (index_);

        CartesianFrame frame (asset_loc);

        ret.frame (frame);

        Location enemy_cart_loc (frame, enemy_loc);

        int even_rank = p.rank % 2 == 0;
        int rank = (even_rank) ? (p.rank / 2) : (- (p.rank + 1) / 2);
        int offset = (p.offset % 2 == 0) ? (p.offset / 2) : (- (p.offset + 1) / 2);

        double distance = asset_loc.distance_to (middle) + rank * buffer_;
        double circ = distance * M_PI * 2;

        double arc_len = offset * buffer_ - (even_rank ? 0 : buffer_ / 2);

        double a = atan2 (enemy_cart_loc.x (), enemy_cart_loc.y ());
        double ao = a + ( ( (arc_len) / circ) * M_PI * 2);
        double xo = sin (ao);
        double yo = cos (ao);
        ret.x (xo * distance);
        ret.y (yo * distance);
        ret.z (0);

        ret.transform_this_to (platform_->get_frame ());
        ret.z (middle.z ());
      }
    }
  }

  return ret;
}

gams::algorithms::ZoneCoverage::formation_func
gams::algorithms::ZoneCoverage::get_form_func (const std::string &form_name)
{
  // default formation
  formation_func result = &ZoneCoverage::line_formation;

  if (form_name.size () != 0)
  {
    switch (form_name[0])
    {
    case 'a':
      if (form_name == "arc")
        result = &ZoneCoverage::arc_formation;
      break;
    case 'o':
      if (form_name == "onion")
        result = &ZoneCoverage::onion_formation;
      break;
    default:
      break;
    }
  }

  return result;
}
