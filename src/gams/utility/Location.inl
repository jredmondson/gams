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

namespace gams
{
  namespace utility
  {
    inline constexpr Location_Vector::Location_Vector(
                            double x, double y, double z)
      : x_(x), y_(y), z_(z) {}

    inline constexpr Location_Vector::Location_Vector()
      : x_(INVAL_COORD), y_(INVAL_COORD), z_(INVAL_COORD) {}

    inline constexpr Location_Vector::Location_Vector(
                            const Location_Vector &orig)
      : x_(orig.x_), y_(orig.y_), z_(orig.z_) { }

    inline constexpr bool Location_Vector::is_invalid() const
    {
      return x_ == INVAL_COORD || y_ == INVAL_COORD || z_ == INVAL_COORD;
    }

    inline constexpr bool
        Location_Vector::operator==(const Location_Vector &other) const
    {
      return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
    }

    inline constexpr bool Location_Vector::is_zero() const
    {
      return x_ == 0 && y_ == 0 && z_ == 0;
    }

    inline std::string Location_Vector::name()
    {
      return "Location";
    }

    inline constexpr double Location_Vector::x() const { return x_; }
    inline constexpr double Location_Vector::y() const { return y_; }
    inline constexpr double Location_Vector::z() const { return z_; }

    inline double Location_Vector::x(double new_x) { return x_ = new_x; }
    inline double Location_Vector::y(double new_y) { return y_ = new_y; }
    inline double Location_Vector::z(double new_z) { return z_ = new_z; }

    inline constexpr double Location_Vector::lng() const { return x_; }
    inline constexpr double Location_Vector::lat() const { return y_; }
    inline constexpr double Location_Vector::alt() const { return z_; }

    inline double Location_Vector::lng(double new_x) { return x_ = new_x; }
    inline double Location_Vector::lat(double new_y) { return y_ = new_y; }
    inline double Location_Vector::alt(double new_z) { return z_ = new_z; }

    inline constexpr double Location_Vector::rho() const { return x_; }
    inline constexpr double Location_Vector::theta() const { return x_; }
    inline constexpr double Location_Vector::phi() const { return y_; }
    inline constexpr double Location_Vector::r() const { return z_; }

    inline double Location_Vector::rho(double new_x) { return x_ = new_x; }
    inline double Location_Vector::theta(double new_x) { return x_ = new_x; }
    inline double Location_Vector::phi(double new_y) { return y_ = new_y; }
    inline double Location_Vector::r(double new_z) { return z_ = new_z; }

    inline constexpr int Location_Vector::size()
    {
      return 3;
    }

    inline constexpr double Location_Vector::get(int i) const
    {
      return i == 0 ? x() :
             i == 1 ? y() :
             i == 2 ? z() :
             throw std::range_error("Index out of bounds for Location");
    }

    inline double Location_Vector::set(int i, double val)
    {
      switch(i)
      {
      case 0:
        return x(val);
      case 1:
        return y(val);
      case 2:
        return z(val);
      default:
        throw std::range_error("Index out of bounds for Location");
      }
    }

    inline Location_Vector &Location_Vector::as_vec()
    {
      return static_cast<Base_Type &>(*this);
    }

    inline constexpr const Location_Vector &Location_Vector::as_vec() const
    {
      return static_cast<const Base_Type &>(*this);
    }

    inline std::ostream &operator<<(std::ostream &o, const Location_Vector &loc)
    {
      o << "(" << loc.x() << "," << loc.y() << "," << loc.z() << ")";
      return o;
    }

    inline Location::Location(double x, double y, double z)
      : Location_Vector(x, y, z), Coordinate() {}

    inline constexpr Location::Location(const Reference_Frame &frame,
                       double x, double y, double z)
      : Location_Vector(x, y, z), Coordinate(frame) {}

    inline Location::Location() : Location_Vector(), Coordinate() {}

    inline constexpr Location::Location(const Location &orig)
      : Location_Vector(orig), Coordinate(orig) {}

    inline Location::Location(const Reference_Frame &new_frame,
                              const Location &orig)
      : Location_Vector(orig), Coordinate(orig.frame())
    {
      transform_this_to(new_frame);
    }
  }
}

#endif
