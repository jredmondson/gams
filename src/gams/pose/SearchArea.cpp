/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
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
 * @file SearchArea.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Search Area is a collection of regions, possibly with priority
 **/

#include "gams/pose/SearchArea.h"

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "gams/pose/Region.h"
#include "gams/loggers/GlobalLogger.h"
#include "madara/utility/Utility.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/String.h"

using std::cerr;
using std::copy;
using std::endl;
using std::max;
using std::set;
using std::sort;
using std::string;
using std::stringstream;
using std::swap;
using std::vector;

namespace mutility = madara::utility;
typedef madara::knowledge::KnowledgeRecord::Integer Integer;

gams::pose::SearchArea::SearchArea() :
  Containerize()
{
}

gams::pose::SearchArea::SearchArea(const PrioritizedRegion& region, 
  const std::string& name) :
  Containerize(name)
{
  regions_.push_back(region);
  calculate_bounding_box();
}

gams::pose::SearchArea::SearchArea(
  const vector<PrioritizedRegion>& regions, const std::string& name) :
  Containerize(name), regions_(regions)
{
  calculate_bounding_box();
}

gams::pose::SearchArea::~SearchArea()
{
}

bool
gams::pose::SearchArea::operator==(const SearchArea& rhs) const
{
  if (this == &rhs)
    return true;

  if (regions_.size() != rhs.regions_.size())
    return false;

  for (const PrioritizedRegion& lpr : regions_)
  {
    bool result = false;
    for (const PrioritizedRegion& rpr : rhs.regions_)
      result |=(lpr != rpr);
    if (!result)
      return false;
  }
  return true;
}

bool
gams::pose::SearchArea::operator!=(const SearchArea& rhs) const
{
  return !(*this == rhs);
}

void
gams::pose::SearchArea::operator=(const SearchArea & rhs)
{
  if (this != &rhs)
  {
    this->regions_ = rhs.regions_;
    this->min_lat_ = rhs.min_lat_;
    this->max_lat_ = rhs.max_lat_;
    this->min_lon_ = rhs.min_lon_;
    this->max_lon_ = rhs.max_lon_;
    this->min_alt_ = rhs.min_alt_;
    this->max_alt_ = rhs.max_alt_;
    this->name_ = rhs.name_;
  }
}

void
gams::pose::SearchArea::add_prioritized_region(const PrioritizedRegion& r)
{
  regions_.push_back(r);

  // modify bounding box
  min_lat_ =(min_lat_ > r.min_lat_) ? r.min_lat_ : min_lat_;
  min_lon_ =(min_lon_ > r.min_lon_) ? r.min_lon_ : min_lon_;
  min_alt_ =(min_alt_ > r.min_alt_) ? r.min_alt_ : min_alt_;

  max_lat_ =(max_lat_ < r.max_lat_) ? r.max_lat_ : max_lat_;
  max_lon_ =(max_lon_ < r.max_lon_) ? r.max_lon_ : max_lon_;
  max_alt_ =(max_alt_ < r.max_alt_) ? r.max_alt_ : max_alt_;
}

gams::pose::Region
gams::pose::SearchArea::get_convex_hull() const
{
  /**
   * completely rewritten by James Edmondson on 4/3/2016.
   * old version harder to maintain, more computationally
   * expensive and unlikely to produce convex hulls consistently
   * due to modifications made using arbitrary insertion of
   * [0,0,0] origins with cross products.
   *
   * The new convex hull uses Andrew's Monotone Chain Algorithm
   * rather than Graham scan and also inlines the cross
   **/

  class SortXThenY
  {
  public:
    bool operator()(const gams::pose::Position & first,
      const gams::pose::Position & second)
    {
      // prefer left-most x or if tie, bottom-most left-most x
      return first.x() < second.x() ||(first.x() == second.x() && first.y() < second.y());
    }
  };

  /**
   * memory allocation is expensive, so only do that once when creating the list
   * of all points
   **/

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::pose::SearchArea::get_convex_hull:" \
    " building a convex hull\n");

  Region result;
  vector<Position> positions;
  vector<Position> & hull(result.vertices);
  size_t num_points(0), current(0), k(0);

  // iterate over all regions 
  for (size_t i = 0; i < regions_.size(); ++i)
  {
    num_points += regions_[i].vertices.size();
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " total region vertices is %d. Resizing list.\n",
   (int)num_points);

  // resize the positions array
  positions.resize(num_points);

  // fill the positions array
  for (size_t i = 0; i < regions_.size(); ++i)
  {
    for (size_t j = 0; j < regions_[i].vertices.size(); ++j, ++current)
    {
      positions[current] = regions_[i].vertices[j];
    }
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " sorting vertices.\n");

  // sort by x and then y
  std::sort(positions.begin(), positions.end(), SortXThenY());

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " removing duplicate vertices.\n");

  // get rid of duplicates
  if (positions.size() > 2)
  {
    size_t vacant = 1;
    for (size_t i = 1; i < positions.size(); ++i)
    {
      // do we have a duplicate?
      if (positions[i] != positions[vacant - 1])
      {
        // yes, shift 
        if (i != vacant)
        {
          positions[vacant] = positions[i];
        }
        ++vacant;
      }
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_DETAILED,
      "gams::pose::SearchArea::get_convex_hull:" \
      " after duplicates check, positions size set to %d.\n",
     (int)vacant);

    // resize positions to just past unique entries
    positions.resize(vacant);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_DETAILED,
      "gams::pose::SearchArea::get_convex_hull:" \
      " allocating %d vertices for potential hull.\n",
     (int)vacant * 2);

    // make hull twice as big for speed in constructing upper and lower hull
    hull.resize(vacant * 2);
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " building lower hull.\n");

  // build lower hull
  for (int i = 0; i <(int)positions.size(); ++i)
  {
    while(k >= 2 && cross(hull[k - 2], hull[k - 1], positions[i]) <= 0)
      k--;

    hull[k++] = positions[i];
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " lower hull construction used %d points.\n",(int)k);

  if (positions.size() >= 2)
  {
    // build upper hull
    for (int i =(int)positions.size() - 2, t =(int)k + 1;
         i >= 0; i--)
    {
      while((int)k >= t && cross(hull[k - 2], hull[k - 1], positions[i]) <= 0)
        k--;

      hull[k++] = positions[i];
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
      "gams::pose::SearchArea::get_convex_hull:" \
      " ERROR: not enough vertices in region for bounding box creation.\n");
  }

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_DETAILED,
    "gams::pose::SearchArea::get_convex_hull:" \
    " upper hull construction finished hull with %d points.\n",(int)k - 1);

  if (k > 0)
  {
    hull.resize(k - 1);
  }
  else
  {
    hull.resize(0);
  }

  return result;
}

const vector<gams::pose::PrioritizedRegion>&
gams::pose::SearchArea::get_regions() const
{
  return regions_;
}

madara::knowledge::KnowledgeRecord::Integer
gams::pose::SearchArea::get_priority(const Position& pos) const
{
  madara::knowledge::KnowledgeRecord::Integer priority = 0;
  for (vector<PrioritizedRegion>::const_iterator it = regions_.begin();
    it != regions_.end(); ++it)
  {
    if (it->contains(pos))
      priority = max(priority, it->priority);
  }

  return priority;
}

bool
gams::pose::SearchArea::contains(const Position & p) const
{
  for (unsigned int i = 0; i < regions_.size(); ++i)
    if (regions_[i].contains(p))
      return true;
  return false;
}

string
gams::pose::SearchArea::to_string() const
{
  stringstream buffer;
  buffer << "Num regions: " << regions_.size() << endl;
  for (unsigned int i = 0; i < regions_.size(); ++i)
    buffer << "Region " << i << ": " << regions_[i].to_string() << endl;
  return buffer.str();
}

void
gams::pose::SearchArea::calculate_bounding_box()
{
  min_lat_ = min_lon_ = min_alt_ = DBL_MAX;
  max_lat_ = max_lon_ = max_alt_ = -DBL_MAX;
  for (unsigned int i = 0; i < regions_.size(); ++i)
  {
    min_lat_ =(min_lat_ > regions_[i].min_lat_) ? regions_[i].min_lat_ : min_lat_;
    min_lon_ =(min_lon_ > regions_[i].min_lon_) ? regions_[i].min_lon_ : min_lon_;
    min_alt_ =(min_alt_ > regions_[i].min_alt_) ? regions_[i].min_alt_ : min_alt_;

    max_lat_ =(max_lat_ < regions_[i].max_lat_) ? regions_[i].max_lat_ : max_lat_;
    max_lon_ =(max_lon_ < regions_[i].max_lon_) ? regions_[i].max_lon_ : max_lon_;
    max_alt_ =(max_alt_ < regions_[i].max_alt_) ? regions_[i].max_alt_ : max_alt_;
  }
}

bool
gams::pose::SearchArea::check_valid_type(
  madara::knowledge::KnowledgeBase& kb, const std::string& name) const
{
  const static Class_ID valid =(Class_ID) 
   (REGION_TYPE_ID        | PRIORITIZED_REGION_TYPE_ID | 
     SEARCH_AREA_TYPE_ID);
  return Containerize::is_valid_type(kb, name, valid);
}

void
gams::pose::SearchArea::to_container_impl(
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  // set object type
  madara::knowledge::containers::Integer obj_type(
    name + object_type_suffix_, kb);
  obj_type = SEARCH_AREA_TYPE_ID;
  
  // set size
  madara::knowledge::containers::Integer size(name + ".size", kb);
  size = regions_.size();

  // add regions
  size_t idx = 0;
  for (PrioritizedRegion& pr : regions_)
  {
    // check if PR has name
    string pr_name = pr.get_name();
    if (pr_name == "")
    {
      // PR has no name, so assign a name
      std::stringstream new_pr_name;
      new_pr_name << name << "." << idx;
      pr_name = new_pr_name.str();
      pr.set_name(pr_name);
    }

    // check if PR is in KB
    const Class_ID valid =(Class_ID) 
     (REGION_TYPE_ID | PRIORITIZED_REGION_TYPE_ID);
    if (!is_valid_type(kb, pr_name, valid))
    {
      // PR must not be in KB, so put it in there
      pr.to_container(kb, pr_name);
    }

    // set PR as member of search area
    std::stringstream member_key;
    member_key << name << "." << idx;
    madara::knowledge::containers::String member(member_key.str(), kb);
    member = pr_name;

    // increment index
    ++idx;
  }
}

bool
gams::pose::SearchArea::from_container_impl(
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  bool ret_val(false);
  if (!check_valid_type(kb, name))
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ERROR,
      "gams::pose::SearchArea::from_container:" \
      " \"%s\" is not a valid SearchArea\n", name.c_str());
  }
  else
  {
    switch(get_type(kb, name))
    {
      case REGION_TYPE_ID:
      case PRIORITIZED_REGION_TYPE_ID:
      {
        PrioritizedRegion reg;
        if (reg.from_container(kb, name))
        {
          regions_.clear();
          regions_.push_back(reg);
          calculate_bounding_box();
          ret_val = true;
        }
        break;
      }
      case SEARCH_AREA_TYPE_ID:
      {
        // get size
        madara::knowledge::containers::Integer size(name + ".size", kb);
        if (size.exists())
        {
          // reserve space in regions_
          regions_.clear();
          regions_.reserve(size.to_integer());
        
          // get regions
          ret_val = true;
          for (Integer idx = 0; idx < size.to_integer() && ret_val; ++idx)
          {
            std::stringstream pr_prefix;
            pr_prefix << name << "." << idx;
        
            PrioritizedRegion temp;
            if (!(ret_val = temp.from_container(
              kb, kb.get(pr_prefix.str()).to_string())))
            {
              madara_logger_ptr_log(gams::loggers::global_logger.get(),
                gams::loggers::LOG_ERROR,
                "gams::pose::SearchArea::from_container:" \
                " \"%s\" is not valid PrioritizedRegion\n",
                pr_prefix.str().c_str());
            }
            else
              regions_.push_back(temp);
          }
        
          if (ret_val)
            calculate_bounding_box();
        }
        else
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_ERROR,
            "gams::pose::SearchArea::from_container:" \
            " \"%s\" does not have size value\n", name.c_str());
        }
        break;
      }
      default:
      {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_ERROR,
          "gams::pose::SearchArea::from_container:" \
          " found invalid object_type %u\n", get_type(kb, name));
      }
    }
  }
  return ret_val;;
}
