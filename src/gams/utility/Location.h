/**
 * Copyright (c) 2015 Carnegie Mellon University. All Rights Reserved.
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
 * @file Coordinates.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location, Rotation, and Pose classes
 **/

#ifndef _GAMS_UTILITY_LOCATION_H_
#define _GAMS_UTILITY_LOCATION_H_

#include "gams/GAMS_Export.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>
#include <cstdio>

#include <gams/utility/Coordinate.h>

namespace gams
{
  namespace utility
  {
    class Base_Frame;

    /**
     * Container for Location information, not bound to a frame.
     * Do not use unless implementing new frames or coordinate types.
     **/
    class GAMS_Export Location_Base
    {
    public:
      Location_Base(double x, double y, double z = 0.0)
        : _x(x), _y(y), _z(z) {}

      Location_Base()
        : _x(INVAL_COORD), _y(INVAL_COORD), _z(INVAL_COORD) {}

      Location_Base(const Location_Base &orig)
        : _x(orig._x), _y(orig._y), _z(orig._z) { }

      bool isInvalid()
      {
        return _x == INVAL_COORD || _y == INVAL_COORD || _z == INVAL_COORD;
      }

      bool operator==(const Location_Base &other) const
      {
        return _x == other._x && _y == other._y && _z == other._z;
      }

      static std::string name()
      {
        return "Location";
      }

      double x() const { return _x; }
      double y() const { return _y; }
      double z() const { return _z; }

      double x(double new_x) { return _x = new_x; }
      double y(double new_y) { return _y = new_y; }
      double z(double new_z) { return _z = new_z; }

      /**
       * Alternative names for x, y, and z, suitable for
       * non-Cartesian coordinate systems. Use:
       *
       * lng/lat/alt for GPS-style systems
       * rho/phi/r for Cylindrical systems
       * theta/phi/r for Spherical systems
       **/
      double lng() const { return _x; }
      double lat() const { return _y; }
      double alt() const { return _z; }

      double lng(double new_x) { return _x = new_x; }
      double lat(double new_y) { return _y = new_y; }
      double alt(double new_z) { return _z = new_z; }

      double rho() const { return _x; }
      double theta() const { return _x; }
      double phi() const { return _y; }
      double r() const { return _z; }

      double rho(double new_x) { return _x = new_x; }
      double theta(double new_x) { return _x = new_x; }
      double phi(double new_y) { return _y = new_y; }
      double r(double new_z) { return _z = new_z; }

      int size() const
      {
        return 3;
      }

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z
       **/
      double get(int i) const
      {
        if(i == 0)
          return x();
        else if(i == 1)
          return y();
        else if(i == 2)
          return z();
        throw std::range_error("Index out of bounds for Location");
      }

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z
       **/
      double set(int i, double val)
      {
        if(i == 0)
          return x(val);
        else if(i == 1)
          return y(val);
        else if(i == 2)
          return z(val);
        throw std::range_error("Index out of bounds for Location");
      }

      typedef Location_Base Base_Type;
    private:
      double _x, _y, _z;
    };

    /**
     * Represents a Location within a reference frame.
     * This location always has x, y, and z coordinates, but interpretation
     * of those coordinates can vary according to the reference frame.
     **/
    class GAMS_Export Location : public Location_Base, public Coordinate<Location>
    {
    public:
      Location(double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Coordinate() {}

      Location(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Coordinate(frame) {}

      Location() : Location_Base(), Coordinate() {}

      Location(const Location &orig)
        : Location_Base(orig), Coordinate(orig) {}

      Location(const Base_Frame &new_frame, const Location &orig)
        : Location_Base(orig), Coordinate(orig.frame())
      {
        transform_this_to(new_frame);
        frame(new_frame);
      }

      using Coordinate<Location>::operator==;
    };
  }
}

#endif
