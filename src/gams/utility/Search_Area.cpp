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
#include "madara/utility/Utility.h"

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

gams::utility::Region
gams::utility::Search_Area::get_convex_hull () const
{
  /**
   * Use Graham Scan algorithm
   * Time complexity is O(n * log n)
   * pseudocode at https://en.wikipedia.org/wiki/Graham_scan
   */
  // get all points, filter out duplicates
  set<GPS_Position> s_points;
  for (size_t i = 0; i < regions_.size (); ++i)
    for (size_t j = 0; j < regions_[i].vertices.size (); ++j)
      s_points.insert (regions_[i].vertices[j]);
  const size_t N = s_points.size ();

  // create array of points
  vector <GPS_Position> points (N + 1);
  vector <GPS_Position>::iterator start = points.begin ();
  ++start;
  copy (s_points.begin (), s_points.end (), start);
//  for (int i = 0; i < N+1; ++i)
//    cerr << std::setprecision(10) << points[i].latitude() << " " << points[i].longitude() << endl;

  // find point with lowest y/lat coord...
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

  // sort positions
  sort (&points[2], &points[N + 1], sort_by_angle (points[1]));
//  cerr << "sorting points" << endl;
//  for (int i = 0; i < N+1; ++i)
//    cerr << points[i].latitude() << " " << points[i].longitude() << endl;

  // copy sentinel point
  points[0] = points[N];

  // find convex hull
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

void
gams::utility::Search_Area::init (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const string & prefix)
{
  // get size of search_area in number of regions
  if (mutility::begins_with (prefix, "search_area"))
  {
    string search_area_prefix (prefix + ".");
    string size_key (prefix + ".size");

    Integer num_regions = knowledge.get (size_key).to_integer ();
  
    // parse each region
    for (unsigned int i = 0; i < num_regions; ++i)
    {
      std::stringstream region;
      region << search_area_prefix << i;

      // get prioritized region and add to search area
      Prioritized_Region p_region;
      p_region.from_container (
        knowledge.get (region.str ()).to_string (), knowledge);
      add_prioritized_region (p_region);
    }
  }
  else // this is just a region
  {
    Prioritized_Region p_region;
    p_region.from_container (prefix, knowledge);
    add_prioritized_region (p_region);
  }
}

gams::utility::Search_Area
gams::utility::parse_search_area (
  Madara::Knowledge_Engine::Knowledge_Base & knowledge,
  const string & prefix)
{
  Search_Area result;
  result.init (knowledge, prefix);
  return result;
}
