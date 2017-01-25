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
 * @file Location.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the inline functions for the Location class
 **/

#ifndef _GAMS_UTILITY_LOCATION_INL_
#define _GAMS_UTILITY_LOCATION_INL_

#include "Location.h"

namespace gams
{
  namespace utility
  {
    inline constexpr LocationVector::LocationVector(
                            double x, double y, double z)
      : x_(x), y_(y), z_(z) {}

    inline constexpr LocationVector::LocationVector (const LocationVector & rhs)
      : x_ (rhs.x_), y_ (rhs.y_), z_ (rhs.z_)
    {
    }

    inline constexpr LocationVector::LocationVector()
      : x_(INVAL_COORD), y_(INVAL_COORD), z_(INVAL_COORD) {}

    inline LocationVector::LocationVector(const double array[])
      : x_(array[0]), y_(array[1]), z_(array[2]) {}

    inline LocationVector::LocationVector(const float array[])
      : x_(array[0]), y_(array[1]), z_(array[2]) {}

    inline LocationVector::LocationVector(
        const madara::knowledge::containers::DoubleVector &vec)
      : x_(vec[0]), y_(vec[1]), z_(vec[2]) {}

    inline LocationVector::LocationVector(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : x_(vec[0]), y_(vec[1]), z_(vec[2]) {}

    inline constexpr bool LocationVector::is_set() const
    {
      return x_ != INVAL_COORD || y_ != INVAL_COORD || z_ != INVAL_COORD;
    }

    inline constexpr bool
        LocationVector::operator==(const LocationVector &other) const
    {
      return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
    }

    inline constexpr bool LocationVector::is_zero() const
    {
      return x_ == 0 && y_ == 0 && z_ == 0;
    }

    inline std::string LocationVector::name()
    {
      return "Location";
    }

    inline constexpr double LocationVector::x() const { return x_; }
    inline constexpr double LocationVector::y() const { return y_; }
    inline constexpr double LocationVector::z() const { return z_; }

    inline double LocationVector::x(double new_x) { return x_ = new_x; }
    inline double LocationVector::y(double new_y) { return y_ = new_y; }
    inline double LocationVector::z(double new_z) { return z_ = new_z; }

    inline constexpr double LocationVector::lng() const { return x_; }
    inline constexpr double LocationVector::lat() const { return y_; }
    inline constexpr double LocationVector::alt() const { return z_; }

    inline double LocationVector::lng(double new_x) { return x_ = new_x; }
    inline double LocationVector::lat(double new_y) { return y_ = new_y; }
    inline double LocationVector::alt(double new_z) { return z_ = new_z; }

    inline constexpr double LocationVector::rho() const { return x_; }
    inline constexpr double LocationVector::theta() const { return x_; }
    inline constexpr double LocationVector::phi() const { return y_; }
    inline constexpr double LocationVector::r() const { return z_; }

    inline double LocationVector::rho(double new_x) { return x_ = new_x; }
    inline double LocationVector::theta(double new_x) { return x_ = new_x; }
    inline double LocationVector::phi(double new_y) { return y_ = new_y; }
    inline double LocationVector::r(double new_z) { return z_ = new_z; }

    inline constexpr int LocationVector::size()
    {
      return 3;
    }

    inline constexpr double LocationVector::get(int i) const
    {
      return i == 0 ? x() :
             i == 1 ? y() :
             i == 2 ? z() :
             throw std::range_error("Index out of bounds for Location");
    }

    inline double LocationVector::set(int i, double val)
    {
      return i == 0 ? x(val) :
             i == 1 ? y(val) :
             i == 2 ? z(val) :
             throw std::range_error("Index out of bounds for Location");
    }

    inline LocationVector &LocationVector::as_vec()
    {
      return static_cast<BaseType &>(*this);
    }

    inline constexpr const LocationVector &LocationVector::as_vec() const
    {
      return static_cast<const BaseType &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const LocationVector &loc)
    {
      o << "(" << loc.x() << "," << loc.y() << "," << loc.z() << ")";
      return o;
    }

    inline std::string Location::to_string (const std::string & delimiter,
      const std::string & unset_identifier) const
    {
      std::stringstream buffer;

      if (x () != DBL_MAX)
        buffer << x ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (y () != DBL_MAX)
        buffer << y ();
      else
        buffer << unset_identifier;

      buffer << delimiter;

      if (z () != DBL_MAX)
        buffer << z ();
      else
        buffer << unset_identifier;

      return buffer.str ();
    }

    inline Location::Location(double x, double y, double z)
      : LocationVector(x, y, z), Coordinate() {}

    inline constexpr Location::Location(const ReferenceFrame &frame,
                       double x, double y, double z)
      : LocationVector(x, y, z), Coordinate(frame) {}

    inline Location::Location() : LocationVector(), Coordinate() {}

    inline Location::Location(const ReferenceFrame &frame)
      : LocationVector(), Coordinate(frame) {}

    inline Location::Location(const ReferenceFrame &new_frame,
                              const Location &orig)
      : LocationVector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }

    inline Location::Location (const Location & rhs)
      : LocationVector (rhs.x (), rhs.y (), rhs.z ()), Coordinate (rhs.frame ())
    {
    }

    inline Location::Location(const double array[])
      : LocationVector(array), Coordinate() {}

    inline Location::Location(const ReferenceFrame &frame, const double array[])
      : LocationVector(array), Coordinate(frame) {}

    inline Location::Location(const float array[])
      : LocationVector(array), Coordinate() {}

    inline Location::Location(const ReferenceFrame &frame, const float array[])
      : LocationVector(array), Coordinate(frame) {}

    inline Location::Location(
        const madara::knowledge::containers::DoubleVector &vec)
      : LocationVector(vec), Coordinate() {}

    inline Location::Location(const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec)
      : LocationVector(vec), Coordinate(frame) {}

    inline Location::Location(
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : LocationVector(vec), Coordinate() {}

    inline Location::Location(
        const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec)
      : LocationVector(vec), Coordinate(frame) {}
  }
}

#endif
