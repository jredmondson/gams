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
 * @file Search_Area.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Search Area is a collection of regions, possibly with priority
 **/

#include "gams/utility/Search_Area.h"

#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "gams/utility/Region.h"
#include "gams/utility/GPS_Position.h"
#include "gams/loggers/Global_Logger.h"
#include "madara/utility/Utility.h"
#include "madara/knowledge_engine/containers/Integer.h"
#include "madara/knowledge_engine/containers/String.h"

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

namespace mutility = Madara::Utility;
typedef Madara::Knowledge_Record::Integer Integer;

gams::utility::Search_Area::Search_Area ()
{
}

gams::utility::Search_Area::Search_Area (const Prioritized_Region& region)
{
  regions_.push_back (region);
  calculate_bounding_box ();
}

gams::utility::Search_Area::Search_Area (
  const vector<Prioritized_Region>& regions) :
  regions_ (regions)
{
  calculate_bounding_box ();
}

gams::utility::Search_Area::~Search_Area ()
{
}

bool
gams::utility::Search_Area::operator== (const Search_Area& rhs) const
{
  if (this == &rhs)
    return true;

  if (regions_.size () != rhs.regions_.size ())
    return false;

  for (const Prioritized_Region& lpr : regions_)
  {
    bool result = false;
    for (const Prioritized_Region& rpr : rhs.regions_)
      result |= (lpr != rpr);
  }
  return true;
}

bool
gams::utility::Search_Area::operator!= (const Search_Area& rhs) const
{
  return !(*this == rhs);
}

void
gams::utility::Search_Area::operator= (const Search_Area & rhs)
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
gams::utility::Search_Area::add_prioritized_region (const Prioritized_Region& r)
{
  regions_.push_back (r);

  // modify bounding box
  min_lat_ = (min_lat_ > r.min_lat_) ? r.min_lat_ : min_lat_;
  min_lon_ = (min_lon_ > r.min_lon_) ? r.min_lon_ : min_lon_;
  min_alt_ = (min_alt_ > r.min_alt_) ? r.min_alt_ : min_alt_;

  max_lat_ = (max_lat_ < r.max_lat_) ? r.max_lat_ : max_lat_;
  max_lon_ = (max_lon_ < r.max_lon_) ? r.max_lon_ : max_lon_;
  max_alt_ = (max_alt_ < r.max_alt_) ? r.max_alt_ : max_alt_;
}

gams::utility::Region
gams::utility::Search_Area::get_convex_hull () const
{
  // sort points by angle with point, for use in get_convex_hull
  struct sort_by_angle
  {
    const gams::utility::GPS_Position anchor;
  
    sort_by_angle (const gams::utility::GPS_Position& p) : anchor (p) {}
  
    bool operator() (const gams::utility::GPS_Position& gp1,
      const gams::utility::GPS_Position& gp2)
    {
      gams::utility::Position p1 = gp1.to_position (anchor);
      gams::utility::Position p2 = gp2.to_position (anchor);
  
      double angle1 = atan2 (p1.x, p1.y) + 2 * M_PI;
      double angle2 = atan2 (p2.x, p2.y) + 2 * M_PI;
      
      if (angle1 == angle2)
        return (anchor.distance_to(gp1) < anchor.distance_to(gp2));
      else
        return (angle1 < angle2);
    }
  };

  /**
   * Use Graham Scan algorithm
   * Time complexity is O(n * log n)
   * pseudocode at https://en.wikipedia.org/wiki/Graham_scan
   */
  // get all points, filter out duplicates
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " get all points, filter out duplicates\n");

  set<GPS_Position> s_points;
  size_t idx = 0;
  for (size_t i = 0; i < regions_.size (); ++i)
  {
    for (size_t j = 0; j < regions_[i].vertices.size (); ++j)
    {
      s_points.insert (regions_[i].vertices[j]);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::utility::Search_Area::get_convex_hull:" \
        " point %u is \"%f,%f,%f\"\n", idx++,
        regions_[i].vertices[j].x, regions_[i].vertices[j].y, regions_[i].vertices[j].z);
    }
  }
  const size_t N = s_points.size ();

  // create array of points
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " create points array\n");

  vector <GPS_Position> points (N + 1);
  vector <GPS_Position>::iterator start = points.begin ();
  ++start;
  copy (s_points.begin (), s_points.end (), start);
//  for (int i = 0; i < N+1; ++i)
//    cerr << std::setprecision(10) << points[i].latitude() << " " << points[i].longitude() << endl;

  // find point with lowest y/lat coord...
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " find lowest y/lat coord\n");
  size_t lowest = 1;
  double min_lat = points[1].latitude (); 
  for (size_t i = 2; i <= N; ++i)
  {
    if (points[i].latitude () < min_lat)
    {
      lowest = i;
      min_lat = points[i].latitude ();
    }
    else if (points[i].latitude () == min_lat)
    {
      // select western-most point
      if (points[i].longitude () < points[lowest].longitude ())
        lowest = i;
    }
  }

  // ...and swap with points[1]
  swap (points[lowest], points[1]);
  //cerr << "selected " << points[1].latitude() << " " << points[1].longitude() << " as lowest" << endl;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " sort %u points\n", N);
  for (size_t i = 2; i < N + 1; ++i)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::utility::Search_Area::get_convex_hull:" \
      " point %u: \"%f,%f,%f\"\n", i, points[i].x, points[i].y, points[i].z);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " anchor point is \"%f,%f,%f\"\n", points[1].x, points[1].y, points[1].z);

  // sort positions
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " sort points\n");
  sort (&points[2], &points[N + 1], sort_by_angle (points[1]));
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " done sorting\n");
//  cerr << "sorting points" << endl;
//  for (int i = 0; i < N+1; ++i)
//    cerr << points[i].latitude() << " " << points[i].longitude() << endl;

  // copy sentinel point
  points[0] = points[N];

  // find convex hull
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " find convex hull\n");
  unsigned int M = 1;
  for (unsigned int i = 2; i <= N; ++i)
  {
    // find next valid point on convex hull
    while (cross (points[M - 1], points[M], points[i]) <= 0)
    {
      if (M > 1)
        M -= 1;
      else if (i == N)
        break;
      else
        i += 1;
    }

    // update M and swap points to the correct place
    M += 1;
    swap (points[M], points[i]);
  }

  // fill vector of points
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::utility::Search_Area::get_convex_hull:" \
    " fill vector of points\n");
  vector<GPS_Position> temp (M);
  copy (&points[1], &points[M + 1], temp.begin ());
//  for (vector<GPS_Position>::iterator it = temp.begin(); it != temp.end(); ++it)
//    cerr << "\t" << it->latitude () << " " << it->longitude () << endl;
  return Region (temp);
}

const vector<gams::utility::Prioritized_Region>&
gams::utility::Search_Area::get_regions () const
{
  return regions_;
}

Madara::Knowledge_Record::Integer
gams::utility::Search_Area::get_priority (const GPS_Position& pos) const
{
  Madara::Knowledge_Record::Integer priority = 0;
  for (vector<Prioritized_Region>::const_iterator it = regions_.begin ();
    it != regions_.end (); ++it)
  {
    if (it->contains (pos))
      priority = max (priority, it->priority);
  }

  return priority;
}

bool
gams::utility::Search_Area::contains (const GPS_Position & p) const
{
  for (unsigned int i = 0; i < regions_.size(); ++i)
    if (regions_[i].contains (p))
      return true;
  return false;
}

string
gams::utility::Search_Area::to_string () const
{
  stringstream buffer;
  buffer << "Num regions: " << regions_.size () << endl;
  for (unsigned int i = 0; i < regions_.size (); ++i)
    buffer << "Region " << i << ": " << regions_[i].to_string () << endl;
  return buffer.str();
}

void
gams::utility::Search_Area::calculate_bounding_box ()
{
  min_lat_ = min_lon_ = min_alt_ = DBL_MAX;
  max_lat_ = max_lon_ = max_alt_ = -DBL_MAX;
  for (unsigned int i = 0; i < regions_.size (); ++i)
  {
    min_lat_ = (min_lat_ > regions_[i].min_lat_) ? regions_[i].min_lat_ : min_lat_;
    min_lon_ = (min_lon_ > regions_[i].min_lon_) ? regions_[i].min_lon_ : min_lon_;
    min_alt_ = (min_alt_ > regions_[i].min_alt_) ? regions_[i].min_alt_ : min_alt_;

    max_lat_ = (max_lat_ < regions_[i].max_lat_) ? regions_[i].max_lat_ : max_lat_;
    max_lon_ = (max_lon_ < regions_[i].max_lon_) ? regions_[i].max_lon_ : max_lon_;
    max_alt_ = (max_alt_ < regions_[i].max_alt_) ? regions_[i].max_alt_ : max_alt_;
  }
}

double
gams::utility::Search_Area::cross (const GPS_Position& gp1, const GPS_Position& gp2,
  const GPS_Position& gp3) const
{
  Position p1 = gp1.to_position (gp3);
  Position p2 = gp2.to_position (gp3);
  // p3 is (0,0) as it is the reference for p1 and p2
  return (p2.y - p1.y) * (0 - p1.x) -
    (p2.x- p1.x) * (0 - p1.y);
}

bool
gams::utility::Search_Area::check_valid_type (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name) const
{
  const static Class_ID valid = (Class_ID) 
    (REGION_TYPE_ID        | PRIORITIZED_REGION_TYPE_ID | 
     SEARCH_AREA_TYPE_ID);
  return Containerize::is_valid_type (kb, name, valid);
}

void
gams::utility::Search_Area::to_container_impl (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name)
{
  // set object type
  Madara::Knowledge_Engine::Containers::Integer obj_type (
    name + object_type_suffix_, kb);
  obj_type = SEARCH_AREA_TYPE_ID;
  
  // set size
  Madara::Knowledge_Engine::Containers::Integer size (name + ".size", kb);
  size = regions_.size ();

  // add regions
  size_t idx = 0;
  for (Prioritized_Region& pr : regions_)
  {
    // check if PR has name
    string pr_name = pr.get_name ();
    if (pr_name == "")
    {
      // PR has no name, so assign a name
      std::stringstream new_pr_name;
      new_pr_name << name << "." << idx;
      pr_name = new_pr_name.str ();
      pr.set_name (pr_name);
    }

    // check if PR is in KB
    const Class_ID valid = (Class_ID) 
      (REGION_TYPE_ID | PRIORITIZED_REGION_TYPE_ID);
    if (!is_valid_type (kb, pr_name, valid))
    {
      // PR must not be in KB, so put it in there
      pr.to_container (kb, pr_name);
    }

    // set PR as member of search area
    std::stringstream member_key;
    member_key << name << "." << idx;
    Madara::Knowledge_Engine::Containers::String member (member_key.str (), kb);
    member = pr_name;

    // increment index
    ++idx;
  }
}

bool
gams::utility::Search_Area::from_container_impl (
  Madara::Knowledge_Engine::Knowledge_Base& kb, const std::string& name)
{
  bool ret_val (false);
  if (!check_valid_type (kb, name))
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::utility::Search_Area::from_container:" \
      " \"%s\" is not a valid Search_Area\n", name.c_str ());
  }
  else
  {
    switch (get_type (kb, name))
    {
      case REGION_TYPE_ID:
      case PRIORITIZED_REGION_TYPE_ID:
      {
        Prioritized_Region reg;
        if (reg.from_container (kb, name))
        {
          regions_.clear ();
          regions_.push_back (reg);
          calculate_bounding_box ();
          ret_val = true;
        }
        break;
      }
      case SEARCH_AREA_TYPE_ID:
      {
        // get size
        Madara::Knowledge_Engine::Containers::Integer size (name + ".size", kb);
        if (size.exists ())
        {
          // reserve space in regions_
          regions_.clear ();
          regions_.reserve (size.to_integer ());
        
          // get regions
          ret_val = true;
          for (Integer idx = 0; idx < size.to_integer () && ret_val; ++idx)
          {
            std::stringstream pr_prefix;
            pr_prefix << name << "." << idx;
        
            Prioritized_Region temp;
            if(!(ret_val = temp.from_container (
              kb, kb.get (pr_prefix.str ()).to_string ())))
            {
              madara_logger_ptr_log (gams::loggers::global_logger.get (),
                gams::loggers::LOG_ERROR,
                "gams::utility::Search_Area::from_container:" \
                " \"%s\" is not valid Prioritized_Region\n",
                pr_prefix.str ().c_str ());
            }
            else
              regions_.push_back (temp);
          }
        
          if (ret_val)
            calculate_bounding_box ();
        }
        else
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_ERROR,
            "gams::utility::Search_Area::from_container:" \
            " \"%s\" does not have size value\n", name.c_str ());
        }
        break;
      }
    }
  }
  return ret_val;;
}
