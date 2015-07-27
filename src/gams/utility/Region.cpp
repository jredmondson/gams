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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
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
 * @file Region.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a utility class for working with regions
 **/

#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "Region.h"
#include "madara/utility/Utility.h"
#include "gams/loggers/Global_Logger.h"

#include "madara/knowledge_engine/containers/Integer.h"

using std::string;
using std::stringstream;
using std::vector;

typedef  Madara::Knowledge_Record::Integer Integer;

gams::utility::Region::Region (const std::vector <GPS_Position> & init_vertices) :
  vertices (init_vertices)
{
  calculate_bounding_box ();
}

gams::utility::Region::~Region ()
{
}

void
gams::utility::Region::operator= (const Region& rhs)
{
  if (this != &rhs)
  {
    this->vertices = rhs.vertices;
    calculate_bounding_box ();
  }
}

bool
gams::utility::Region::operator== (const Region& rhs) const
{
  if (vertices.size () != rhs.vertices.size ())
    return false;

  // ensure all contents are the same
  for (size_t i = 0; i < vertices.size (); ++i)
  {
    size_t j;
    for (j = 0; j < vertices.size (); ++j)
      if (vertices[i] == rhs.vertices[j])
        break;
    if (j == vertices.size())
      return false;
  }

  return true;
}

bool
gams::utility::Region::contains (const GPS_Position & p) const
{
  // check if in bounding box
  if (p.latitude () < min_lat_ || p.latitude () > max_lat_ ||
      p.longitude () < min_lon_ || p.longitude () > max_lon_)
  {
    return false;
  }

  // check if point in polygon code from 
  // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
  size_t i, j;
  bool ret = false;
  for (i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
  {
    if ( (vertices[i].longitude () > p.longitude ()) !=
        (vertices[j].longitude () > p.longitude ()))
    {
      if (p.latitude () < (vertices[j].latitude () - vertices[i].latitude ()) * (p.longitude () - vertices[i].longitude ()) / 
                (vertices[j].longitude () - vertices[i].longitude ()) + 
                 vertices[i].latitude ())
      {
        ret = !ret;
      }
    }
  }

  // check if this is a vertex point
  if (!ret)
  {
    for (unsigned int i = 0; i < vertices.size() && !ret; ++i)
      ret = (vertices[i] == p);
  }

  // TODO: add check for border point

  return ret;
}

double
gams::utility::Region::distance (const GPS_Position& p) const
{
  // if point is in region, then the distance is 0
  if (contains (p))
    return 0;

  // convert to cartesian coords with equirectangular projection
  const GPS_Position sw (min_lat_, min_lon_);
  vector<Position> local_vertices;
  for (size_t i = 0; i < vertices.size (); ++i)
    local_vertices.push_back (vertices[i].to_position (sw));
  Position local_p = p.to_position (sw);

  // else we check for distance from each edge
  double min_dist = DBL_MAX;
  for (size_t i = 0; i < local_vertices.size (); ++i)
  {
    const size_t i_1 = (i + 1) % local_vertices.size();
    double dist = local_vertices[i].distance_to_2d (local_vertices[i_1], local_p);
    if (dist < min_dist)
      min_dist = dist;
  }

  return min_dist;
}

bool
gams::utility::Region::contains (const Position & p,
  const GPS_Position & ref) const
{
  // convert to GPS_Position and then just use other contains
  return contains (
    utility::GPS_Position::to_gps_position (p, ref));
}

gams::utility::Region
gams::utility::Region::get_bounding_box () const
{
  Region ret;

  GPS_Position p;

  p.latitude (min_lat_);
  p.longitude (min_lon_);
  p.altitude (0);
  ret.vertices.push_back (p);

  p.latitude (min_lat_);
  p.longitude (max_lon_);
  p.altitude (0);
  ret.vertices.push_back (p);

  p.latitude (max_lat_);
  p.longitude (max_lon_);
  p.altitude (0);
  ret.vertices.push_back (p);

  p.latitude (max_lat_);
  p.longitude (min_lon_);
  p.altitude (0);
  ret.vertices.push_back (p);

  ret.min_lat_ = this->min_lat_;
  ret.max_lat_ = this->max_lat_;
  ret.min_lon_ = this->min_lon_;
  ret.max_lon_ = this->max_lon_;
  ret.min_alt_ = this->min_alt_;
  ret.max_alt_ = this->max_alt_;

  return ret;
}

double
gams::utility::Region::get_area () const
{
  if (vertices.size() < 3)
    return 0; // degenerate polygon

  // see http://geomalgorithms.com/a01-_area.html
  // Convert all units to cartesian
  vector<Position> cart_vertices;
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    Position p = vertices[i].to_position (vertices[0]);
    cart_vertices.push_back (p);
  }

  // perform calculations with cartesian vertices
  double area = 0.0;
  size_t i, j, k;
  size_t num_vertices = cart_vertices.size ();
  for (i = 1, j = 2, k = 0; i < num_vertices; ++i, ++j, ++k)
  {
    area += cart_vertices[i].x *
      (cart_vertices[j % num_vertices].y - cart_vertices[k].y);
  }
  area += cart_vertices[0].x * (cart_vertices[1].y - cart_vertices[num_vertices - 1].y);
  return fabs(area / 2);
}

string
gams::utility::Region::to_string (const string & delimiter) const
{
  stringstream buffer;

  if (vertices.size () > 0)
  {
    buffer << vertices[0].to_string ();
    for (unsigned int i = 1; i < vertices.size (); ++i)
    {
      buffer << delimiter;
      buffer << vertices[i].to_string ();
    }
  }

  return buffer.str ();
}

void
gams::utility::Region::to_container (const std::string& name, 
  Madara::Knowledge_Engine::Knowledge_Base& kb) const
{
  // generate prefix
  std::string prefix = std::string ("region.") + name;

  // set type
  Madara::Knowledge_Engine::Containers::Integer type (prefix + ".type", kb);
  type = 0; // only type right now

  // set size
  Madara::Knowledge_Engine::Containers::Integer size (prefix + ".size", kb);
  size = vertices.size();

  // set vertices
  Madara::Knowledge_Engine::Containers::Native_Double_Vector target;
  for (unsigned int i = 0; i < vertices.size (); ++i)
  {
    std::stringstream name;
    name << prefix << "." << i;
    target.set_name (name.str (), kb);
    target.set (2, vertices[i].z);
    target.set (0, vertices[i].x);
    target.set (1, vertices[i].y);
  }
}

void
gams::utility::Region::from_container (const std::string& name, 
  Madara::Knowledge_Engine::Knowledge_Base& kb)
{
  // generate prefix
  std::string prefix = std::string ("region.") + name;

  // get type
  Madara::Knowledge_Engine::Containers::Integer type (prefix + ".type", kb);
  type_ = type.to_integer ();

  // get size
  Madara::Knowledge_Engine::Containers::Integer size_container (prefix + ".size", kb);
  unsigned int size = size_container.to_integer ();

  // get vertices
  vertices.resize (size);
  Madara::Knowledge_Engine::Containers::Native_Double_Vector vertex;
  for (unsigned int i = 0; i < size; ++i)
  {
    std::stringstream name;
    name << prefix << "." << i;
    vertex.set_name (name.str (), kb);
    vertices[i].x = vertex[0];
    vertices[i].y = vertex[1];
    vertices[i].z = vertex[2];
  }

  calculate_bounding_box ();
}

void
gams::utility::Region::calculate_bounding_box ()
{
  min_lat_ = DBL_MAX;
  min_lon_ = DBL_MAX;
  min_alt_ = DBL_MAX;
  max_lat_ = -DBL_MAX;
  max_lon_ = -DBL_MAX;
  max_alt_ = -DBL_MAX;
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    min_lat_ = (min_lat_ > vertices[i].latitude ()) ?
      vertices[i].latitude () : min_lat_;
    min_lon_ = (min_lon_ > vertices[i].longitude ()) ?
      vertices[i].longitude () : min_lon_;
    min_alt_ = (min_alt_ > vertices[i].altitude ()) ?
      vertices[i].altitude () : min_alt_;

    max_lat_ = (max_lat_ < vertices[i].latitude ()) ?
      vertices[i].latitude () : max_lat_;
    max_lon_ = (max_lon_ < vertices[i].longitude ()) ?
      vertices[i].longitude () : max_lon_;
    max_alt_ = (max_alt_ < vertices[i].altitude ()) ?
      vertices[i].altitude () : max_alt_;
  }
}

void
gams::utility::Region::init (
  Madara::Knowledge_Engine::Knowledge_Base& knowledge,
  const string & prefix)
{
  // get region id
  string region_prefix (prefix + ".");
  string region_size = prefix + ".size";

  vertices.clear ();

  // building the region type identifier
  //sprintf (expression, "region.%u.type", region_id);
  std::stringstream region_type_str;
  region_type_str << region_prefix;
  region_type_str << "type";

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::utility::Region::init:" \
    " reading type from %s.\n", region_type_str.str ().c_str ());

  Integer region_type = knowledge.get (region_type_str.str ()).to_integer ();
  switch (region_type)
  {
    case 0: // arbitrary convex polygon
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::utility::Region::init:" \
        " type is arbitrary convex polygon.\n");

      const Integer num_vertices = knowledge.get (region_size).to_integer ();
      for (Integer i = 0; i < num_vertices; ++i) // get the vertices
      {
        std::stringstream vertex_name;
        vertex_name << region_prefix << i;

        Madara::Knowledge_Record record = knowledge.get (vertex_name.str ());

        std::vector <double> coords = record.to_doubles ();

        if (coords.size () == 2)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
             "gams::utility::Region::init:" \
            " Adding coordinate (%d, %d).\n",
            coords[0], coords[1]);
          vertices.push_back (GPS_Position(coords[0], coords[1]));
        }
        else if (coords.size () == 3)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
             "gams::utility::Region::init:" \
            " Adding coordinate (%d, %d, %d).\n",
            coords[0], coords[1], coords[2]);
          vertices.push_back (GPS_Position(coords[0], coords[1], coords[2]));
        }
        else
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_ERROR,
             "gams::utility::Region::init:" \
            " ERROR: invalid coordinate type at %s.\n", vertex_name.str ().c_str ());
        }
      }
      break;
    }
    default:
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::utility::Region::init:" \
        " ERROR: invalid region type %" PRId64 ".\n", region_type);
  }

  // recalculate the bounding box
  calculate_bounding_box ();
}

gams::utility::Region gams::utility::parse_region (
  Madara::Knowledge_Engine::Knowledge_Base& knowledge,
  const std::string & prefix)
{
  Region result;
  result.init (knowledge, prefix);
  return result;
}
