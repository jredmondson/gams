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
#include "gams/loggers/GlobalLogger.h"

#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/Vector.h"

using std::string;
using std::stringstream;
using std::vector;

typedef  madara::knowledge::KnowledgeRecord::Integer Integer;

gams::pose::Region::Region (
  const std::vector <Position> & init_vertices, unsigned int type, 
  const std::string& name) :
  Containerize (name), vertices (init_vertices), type_ (type)
{
  set_name (name);
  calculate_bounding_box ();
  for (Position& pos : vertices)
  {
    pos.transform_this_to(pose::gps_frame());
  }
}

gams::pose::Region::~Region ()
{
}

void
gams::pose::Region::operator= (const Region& rhs)
{
  if (this != &rhs)
  {
    this->vertices = rhs.vertices;
    this->name_ = rhs.name_;
    this->type_ = rhs.type_;
    calculate_bounding_box ();
  }
}

bool
gams::pose::Region::operator== (const Region& rhs) const
{
  if (this == &rhs)
    return true;

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
gams::pose::Region::operator!= (const Region& rhs) const
{
  return !(*this == rhs);
}

std::string
gams::pose::Region::get_name () const
{
  return name_;
}

void
gams::pose::Region::set_name (const std::string& n)
{
  name_ = n;
}

bool
gams::pose::Region::contains (const Position & pos) const
{
  if(vertices.size() < 1)
  {
    return false;
  }

  Position p(pose::gps_frame(), pos);

  // check if in bounding box
  if (p.latitude () < min_lat_ || p.latitude () > max_lat_ ||
      p.longitude () < min_lon_ || p.longitude () > max_lon_)
  {
    return false;
  }

  // check if point in polygon code from 
  // http://www.ecse.rpi.edu/Homepages/wrf/Research/ShortNotes/pnpoly.html
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
gams::pose::Region::distance (const Position& p) const
{
  if (vertices.size() < 1)
    return DBL_MAX;
  // if point is in region, then the distance is 0
  if (contains (p))
    return 0;

  // convert to cartesian coords with equirectangular projection
  const Position sw (pose::gps_frame(), min_lon_, min_lat_);
  CartesianFrame local_frame(sw);

  vector<Position> local_vertices;
  for (size_t i = 0; i < vertices.size (); ++i)
  {
    local_vertices.push_back (vertices[i].transform_to (local_frame));
    local_vertices.back().z(0);
  }
  Position local_p = p.transform_to (local_frame);
  local_p.z(0);

  // else we check for distance from each edge
  double min_dist = DBL_MAX;
  for (size_t i = 0; i < local_vertices.size (); ++i)
  {
    //const size_t i_1 = (i + 1) % local_vertices.size();
    //double dist = local_vertices[i].distance_to (local_vertices[i_1], local_p);

    // TODO: Calculate distance to edge instead of to vertex
    double dist = local_vertices[i].distance_to (local_p);
    if (dist < min_dist)
      min_dist = dist;
  }

  return min_dist;
}

gams::pose::Region
gams::pose::Region::get_bounding_box () const
{
  Region ret;

  Position p;

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
gams::pose::Region::get_area () const
{
  if (vertices.size() < 3)
    return 0; // degenerate polygon

  // convert to cartesian coords with equirectangular projection
  const Position sw (pose::gps_frame(), min_lon_, min_lat_);
  CartesianFrame local_frame(sw);
  vector<Position> cart_vertices;
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    Position p = vertices[i].transform_to (local_frame);
    cart_vertices.push_back (p);
  }

  // perform calculations with cartesian vertices
  double area = 0.0;
  size_t i, j, k;
  size_t num_vertices = cart_vertices.size ();
  for (i = 1, j = 2, k = 0; i < num_vertices; ++i, ++j, ++k)
  {
    area += cart_vertices[i].x() *
      (cart_vertices[j % num_vertices].y() - cart_vertices[k].y());
  }
  area += cart_vertices[0].x() * (cart_vertices[1].y() - cart_vertices[num_vertices - 1].y());
  return fabs(area / 2);
}

string
gams::pose::Region::to_string (const string & delimiter) const
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
gams::pose::Region::calculate_bounding_box ()
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

bool
gams::pose::Region::check_valid_type (
  madara::knowledge::KnowledgeBase& kb, const std::string& name) const
{
  const static Class_ID valid = 
    (Class_ID) (REGION_TYPE_ID | PRIORITIZED_REGION_TYPE_ID);
  return Containerize::is_valid_type (kb, name, valid);
}

void
gams::pose::Region::to_container_impl (
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  // set object type
  madara::knowledge::containers::Integer obj_type (
    name + object_type_suffix_, kb);
  obj_type = REGION_TYPE_ID;
  
  // set type of region
  madara::knowledge::containers::Integer type (name + ".type", kb);
  type = type_;

  switch (type.to_integer ())
  {
    case 0: // arbitrary convex polygon
    {
      // set size
      madara::knowledge::containers::Integer size (name + ".size", kb);
      size = vertices.size();
    
      // set vertices
      madara::knowledge::containers::NativeDoubleVector target;
      for (unsigned int i = 0; i < vertices.size (); ++i)
      {
        std::stringstream vert_name;
        vert_name << name << "." << i;
        target.set_name (vert_name.str (), kb);
        target.set (2, vertices[i].z());
        target.set (0, vertices[i].lat());
        target.set (1, vertices[i].lng());
      }

      break;
    }
    default:
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::pose::Region::to_container:" \
        " ERROR: invalid region type %" PRId64 ".\n", type.to_integer ());
  }
}

bool
gams::pose::Region::from_container_impl (
  madara::knowledge::KnowledgeBase& kb, const std::string& name)
{
  if (!check_valid_type (kb, name))
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::pose::Region::from_container:" \
      " \"%s\" is not a valid Region\n", name.c_str ());
    return false;
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::pose::Region::from_container:" \
    " name = %s\n", name.c_str ());

  // get type
  madara::knowledge::KnowledgeRecord type = kb.get (name + ".type");
  if (!type.exists ())
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::pose::Region::from_container:" \
      " \"%s.type\" does not exist in knowledge base\n", name.c_str ());
    return false;
  }
  type_ = type.to_integer ();

  // set name if necessary
  if (name_ == "")
    name_ = name;

  // get vertices
  switch (type_)
  {
    case 0: // arbitrary convex polygon
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
         "gams::pose::Region::from_container:" \
         " type is arbitrary convex polygon\n");

      // get size
      madara::knowledge::KnowledgeRecord num_verts = kb.get (name + ".size");
      if (!num_verts.exists ())
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
          "gams::pose::Region::from_container:" \
          " \"%s.size\" does not exist in knowledge base\n", name.c_str ());
        return false;
      }
      Integer num = num_verts.to_integer ();
      vertices.resize (num);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::pose::Region::from_container:" \
        " size is %u\n", num);

      // get each of the vertices
      madara::knowledge::containers::Vector vertices_knowledge;
      vertices_knowledge.set_name (name, kb);
      vertices_knowledge.resize();
      for (Integer i = 0; i < num; ++i)
      {
        std::vector<double> coords (vertices_knowledge[i].to_doubles ());

        if (coords.size () == 2)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::pose::Region::from_container:" \
            " Adding coordinate (%f lat, %f lng)\n",
            coords[0], coords[1]);
          vertices[i] = Position(pose::gps_frame(), coords[1], coords[0]);
        }
        else if (coords.size () == 3)
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_DETAILED,
            "gams::pose::Region::from_container:" \
            " Adding coordinate (%f lat, %f lng, %f alt)\n",
            coords[0], coords[1], coords[2]);
          vertices[i] = Position(pose::gps_frame(), coords[1], coords[0], coords[2]);
        }
        else
        {
          madara_logger_ptr_log (gams::loggers::global_logger.get (),
            gams::loggers::LOG_ERROR,
            "gams::pose::Region::from_container:" \
            " ERROR: invalid coordinate type at %s.%u\n", name.c_str (), i);
          return false;
        }
      }

      calculate_bounding_box ();

      break;
    }
    default:
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::pose::Region::from_container:" \
        " ERROR: invalid region type %" PRId64 ".\n", type_);
      return false;
    }
  }

  return true;
}
