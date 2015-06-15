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

#ifndef _GAMS_UTILITY_COORDINATES_H_
#define _GAMS_UTILITY_COORDINATES_H_

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

#define DEG_TO_RAD(x) (((x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) (((x) * 180) / M_PI)

#define INVAL_COORD DBL_MAX

namespace gams
{
  namespace utility
  {
    class Base_Frame;

    namespace __INTERNAL__
    {
      extern Base_Frame *default_frame;
    }
    /**
     * New coordinate types which are frame-dependant can inherit from this
     * class. Pass the type of the child class as CoordType
     *
     * New coordinate types must:
     *   -- Inherit from Frame_Bound, and pass itself as template parameter
     *   -- Inherit from a base class which does not inherit Frame_Bound
     *   -- That base class must:
     *      -- Have a typedef Base_Type which refers to itself
     *      -- Have this function: static std::string name()
     **/
    template<typename CoordType>
    class GAMS_Export Frame_Bound
    {
    public:

      Frame_Bound() : _frame(__INTERNAL__::default_frame) {}

      explicit Frame_Bound(const Base_Frame &frame) : _frame(&frame) {}

      explicit Frame_Bound(const Base_Frame *frame) : _frame(frame) {}

      Frame_Bound(const Frame_Bound &orig) : _frame(&orig.frame()) {}

    private:
      const Base_Frame *_frame;

      CoordType &as_coord_type()
      {
        return static_cast<CoordType &>(*this);
      }

      const CoordType &as_coord_type() const
      {
        return static_cast<const CoordType &>(*this);
      }

      template<typename Type>
      Type &as_type()
      {
          return static_cast<Type &>(as_coord_type());
      }

      template<typename Type>
      const Type &as_type() const
      {
          return static_cast<const Type &>(as_coord_type());
      }

    public:
      const Base_Frame &frame() const { return *_frame; }
      const Base_Frame &frame(const Base_Frame &new_frame) {
        return *(_frame = &new_frame);
      }

      // Defined in Base_Frame.h
      CoordType transform_to(const Base_Frame &new_frame) const;
      void transform_this_to(const Base_Frame &new_frame);
      double distance_to(const CoordType &target) const;
      void normalize();

      bool operator==(const CoordType &other) const
      {
        if(frame() == other.frame())
        {
          return as_type<typename CoordType::Base_Type>() == other;
        }
        else
        {
          CoordType tmp(frame(), static_cast<const CoordType &>(other));
          return as_type<typename CoordType::Base_Type>() == tmp;
        }
      }

      bool operator!=(const CoordType &other) const
      {
        return !(*this == other);
      }

      bool approximately_equal(const CoordType &other, double epsilon) const
      {
        return distance_to(other) <= epsilon;
      }

      /**
       * Less than used for ordering in stl containers
       * @param rhs   comparing position
       * @return true if *this is less than rhs
       **/
      bool operator<(const Frame_Bound<CoordType> &rhs) const
      {
        const CoordType &s = as_coord_type();
        const CoordType &o = rhs.as_coord_type();

        if(s.x() < o.x())
          return true;
        if(s.y() < o.y())
          return true;
        return s.z() < o.z();
      }

      std::string to_string(const std::string &delim = ",") const
      {
        std::stringstream buffer;
        const CoordType &s = as_coord_type();
        for(int i = 0; i < s.cardinality(); ++i)
        {
          if(i > 0)
            buffer << delim;
          buffer << s.get(i);
        }
        return buffer.str();
      }

    private:
      static std::istream &skip_nonnum(std::istream &s)
      {
        int next;
        while((next = s.peek()) != EOF)
        {
          if(next == '.' || next == '-' || (next >= '0' && next <= '9'))
            break;
          s.get();
        }
        return s;
      }
    public:

      /**
       * Sets coordinates from a string encoding a sequence of doubles,
       * separated by any set of characters other than 0-9, '.', and '-'
       **/
      void from_string(const std::string &in)
      {
        std::stringstream buffer(in);
        CoordType &s = as_coord_type();
        for(int i = 0; i < s.cardinality(); ++i)
        {
          double val;
          buffer >> skip_nonnum;
          buffer >> val;
          s.set(i, val);
        }
      }

      template<typename ContainType>
      void to_container(ContainType &container) const
      {
        const CoordType &s = as_coord_type();
        for(int i = 0; i < s.cardinality(); i++)
        {
          container.set(i, s.get(i));
        }
      }

      template<typename ContainType>
      void from_container(ContainType &container)
      {
        CoordType &s = as_coord_type();
        for(int i = 0; i < s.cardinality(); i++)
        {
          s.set(i, container[i]);
        }
      }
    };

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

      int cardinality() const
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
    class GAMS_Export Location : public Location_Base, public Frame_Bound<Location>
    {
    public:
      Location(double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Frame_Bound() {}

      Location(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Location_Base(x, y, z), Frame_Bound(frame) {}

      Location() : Location_Base(), Frame_Bound() {}

      Location(const Location &orig)
        : Location_Base(orig), Frame_Bound(orig) {}

      Location(const Base_Frame &new_frame, const Location &orig)
        : Location_Base(orig), Frame_Bound(orig.frame())
      {
        transform_this_to(new_frame);
        frame(new_frame);
      }

      using Frame_Bound<Location>::operator==;
    };

    class Quaternion;

    /**
     * Container for Rotation information, not bound to a frame.
     * Do not use unless implementing new frames or coordinate types.
     **/
    class GAMS_Export Rotation_Base
    {
    public:
      static const int X_axis = 0;
      static const int Y_axis = 1;
      static const int Z_axis = 2;

      Rotation_Base(double rx, double ry, double rz)
        : _rx(rx), _ry(ry), _rz(rz) {}

      Rotation_Base(double x, double y, double z, double angle)
        : _rx(x * angle), _ry(y * angle), _rz(z * angle) {}

      Rotation_Base(int axis_index, double angle)
        : _rx(axis_index == X_axis ? DEG_TO_RAD(angle) : 0),
          _ry(axis_index == Y_axis ? DEG_TO_RAD(angle) : 0),
          _rz(axis_index == Z_axis ? DEG_TO_RAD(angle) : 0) {}

      Rotation_Base()
        : _rx(INVAL_COORD), _ry(INVAL_COORD), _rz(INVAL_COORD) {}

      Rotation_Base(const Rotation_Base &orig)
        : _rx(orig._rx), _ry(orig._ry), _rz(orig._rz) {}

      explicit Rotation_Base(const Quaternion &quat);

      bool isInvalid() {
        return _rx == INVAL_COORD || _ry == INVAL_COORD || _rz == INVAL_COORD;
      }

      static std::string name()
      {
        return "Rotation";
      }

      double rx() const { return _rx; }
      double ry() const { return _ry; }
      double rz() const { return _rz; }

      double rx(double new_rx) { return _rx = new_rx; }
      double ry(double new_ry) { return _ry = new_ry; }
      double rz(double new_rz) { return _rz = new_rz; }

      typedef Rotation_Base Base_Type;

      int cardinality() const
      {
        return 3;
      }

      /**
       * Retrives i'th coordinate, 0-indexed, in order rx, ry, rz
       **/
      double get(int i) const
      {
        if(i == 0)
          return rx();
        else if(i == 1)
          return ry();
        else if(i == 2)
          return rz();
        throw std::range_error("Index out of bounds for Rotation");
      }

      /**
       * Sets i'th coordinate, 0-indexed, in order rx, ry, rz
       **/
      double set(int i, double val)
      {
        if(i == 0)
          return rx(val);
        else if(i == 1)
          return ry(val);
        else if(i == 2)
          return rz(val);
        throw std::range_error("Index out of bounds for Rotation");
      }

      friend class Quaternion;
    private:
      double _rx, _ry, _rz;
    };

    /**
     * Represents a rotation or orientation within a reference frame.
     **/
    class GAMS_Export Rotation : public Rotation_Base, public Frame_Bound<Rotation>
    {
    public:
      /**
       * The direction of rx, ry, and rz is the angle of rotation, with respect
       * to the owning frame's axes.
       * The magnitude of rx, ry, and rz (as a vector) is the amount of rotation,
       * in radians.
       **/
      Rotation(double rx, double ry, double rz)
        : Rotation_Base(rx, ry, rz), Frame_Bound() {}

      Rotation(const Base_Frame &frame, double rx, double ry, double rz)
        : Rotation_Base(rx, ry, rz), Frame_Bound(frame) {}

      /**
       * (x, y, z) must be a unit vector in the direction of rotation axis.
       * angle is the angle of rotation, in radians, about that axis
       **/
      Rotation(double x, double y, double z, double angle)
        : Rotation_Base(x, y, z, angle), Frame_Bound() {}

      Rotation(const Base_Frame &frame, double x, double y, double z, double angle)
        : Rotation_Base(x, y, z, angle), Frame_Bound(frame) {}

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       **/
      Rotation(int axis_index, double angle)
        : Rotation_Base(axis_index, angle), Frame_Bound() {}

      Rotation(const Base_Frame &frame, int axis_index, double angle)
        : Rotation_Base(axis_index, angle), Frame_Bound(frame) {}

      Rotation() : Rotation_Base(), Frame_Bound() {}

      Rotation(const Rotation &orig)
        : Rotation_Base(orig), Frame_Bound(orig) {}

      explicit Rotation(const Quaternion &quat)
        : Rotation_Base(quat) {}
    };

    /**
     * Used internally to implement angle operations.
     * Not reference-frame aware.
     **/
    class GAMS_Export Quaternion
    {
    public:
      Quaternion(double x, double y, double z, double w)
        : _x(x), _y(y), _z(z), _w(w) {}

      Quaternion(double rx, double ry, double rz)
      {
        from_rotation_vector(rx, ry, rz);
      }

      explicit Quaternion(const Rotation_Base &rot)
      {
        from_rotation_vector(rot);
      }

      void from_rotation_vector(double rx, double ry, double rz)
      {
        double magnitude = sqrt(rx * rx + ry * ry + rz * rz);
        if(magnitude == 0)
        {
          _w = 1;
          _x = _y = _z = 0;
        }
        else
        {
          double half_mag = magnitude / 2;
          double cos_half_mag = cos(half_mag);
          double sin_half_mag = sin(half_mag);
          _w = cos_half_mag;
          _x = (rx / magnitude) * sin_half_mag;
          _y = (ry / magnitude) * sin_half_mag;
          _z = (rz / magnitude) * sin_half_mag;
        }
      }

      void from_rotation_vector(const Rotation_Base &rot)
      {
        from_rotation_vector(rot.rx(), rot.ry(), rot.rz());
      }

      void to_rotation_vector(double &rx, double &ry, double &rz) const
      {
        double norm = sqrt(_x * _x + _y * _y + _z * _z);
        double angle = 2 * atan2(norm, _w);
        double sin_half_angle = sin(angle / 2);
        if(angle == 0)
        {
          rx = ry = rz = 0;
        }
        else
        {
          rx = (_x / sin_half_angle) * angle;
          ry = (_y / sin_half_angle) * angle;
          rz = (_z / sin_half_angle) * angle;
        }
      }

      void to_rotation_vector(Rotation_Base &rot) const
      {
        to_rotation_vector(rot._rx, rot._ry, rot._rz);
      }

      Quaternion &operator*=(const Quaternion &o)
      {
        double A = (_w + _x) * (o._w + o._x),
               B = (_z - _y) * (o._y - o._z),
               C = (_w - _x) * (o._y + o._z), 
               D = (_y + _z) * (o._w - o._x),
               E = (_x + _z) * (o._x + o._y),
               F = (_x - _z) * (o._x - o._y),
               G = (_w + _y) * (o._w - o._z),
               H = (_w - _y) * (o._w + o._z);

        _w = B + (-E - F + G + H) / 2;
        _x = A - ( E + F + G + H) / 2;
        _y = C + ( E - F + G - H) / 2;
        _z = D + ( E - F - G + H) / 2;

        return *this;
      }

      Quaternion operator*(const Quaternion &o) const
      {
        Quaternion ret(*this);
        ret *= o;
        return ret;
      }

      Quaternion &conjugate()
      {
        _x = -_x;
        _y = -_y;
        _z = -_z;
        return *this;
      }

      Quaternion operator-() const
      {
        Quaternion ret(-_x, -_y, -_z, _w);
        return ret;
      }

      double inner_product(const Quaternion &o) const
      {
        return _w * o._w + _x * o._x + _y * o._y + _z * o._z;
      }

      double angle_to(const Quaternion &o) const
      {
        double prod = inner_product(o);
        return acos(2 * prod * prod - 1);
      }

      double x() const { return _x; }
      double y() const { return _y; }
      double z() const { return _z; }
      double w() const { return _w; }

      double x(double new_x) { return _x = new_x; }
      double y(double new_y) { return _y = new_y; }
      double z(double new_z) { return _z = new_z; }
      double w(double new_w) { return _w = new_w; }

    private:
      double _x, _y, _z, _w;
    };

    inline Rotation_Base::Rotation_Base(const Quaternion &quat)
    {
      quat.to_rotation_vector(_rx, _ry, _rz);
    }

    inline std::ostream &operator<<(std::ostream &o, const Quaternion &quat)
    {
      o << quat.w() << "+" << quat.x() << "i+" << quat.y() << "j+" << quat.z() << "z";
      return o;
    }

    /**
     * Container for Pose information, not bound to a frame.
     * Do not use unless implementing new frames or coordinate types.
     **/
    class GAMS_Export Pose_Base : public Location_Base, public Rotation_Base
    {
    public:
      Pose_Base(double x, double y, double z, double rx, double ry, double rz)
        : Location_Base(x, y, z), Rotation_Base(rx, ry, rz) {}

      Pose_Base(const Location_Base &loc)
        : Location_Base(loc), Rotation_Base(0, 0, 0) {}

      Pose_Base(const Rotation_Base &rot)
        : Location_Base(0, 0, 0), Rotation_Base(rot) {}

      Pose_Base(const Location_Base &loc, const Rotation_Base &rot)
        : Location_Base(loc), Rotation_Base(rot) {}

      Pose_Base() : Location_Base(), Rotation_Base() {}

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }

      static std::string name()
      {
        return "Pose";
      }

      int cardinality() const
      {
        return 6;
      }

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double get(int i) const
      {
        if(i <= 2)
          return static_cast<const Location_Base &>(*this).get(i);
        else
          return static_cast<const Rotation_Base &>(*this).get(i - 3);
      }

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       **/
      double set(int i, double val)
      {
        if(i <= 2)
          return static_cast<Location_Base &>(*this).set(i, val);
        else
          return static_cast<Rotation_Base &>(*this).set(i - 3, val);
      }

      typedef Pose_Base Base_Type;
    };

    /**
     * Represents a combination of Location and Rotation within a single
     * reference frame.
     **/
    class GAMS_Export Pose : public Pose_Base, public Frame_Bound<Pose>
    {
    public:
      Pose(double x, double y, double z, double rx, double ry, double rz)
        : Pose_Base(x, y, z, rx, ry, rz), Frame_Bound() {}

      Pose(double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Frame_Bound() {}

      Pose(const Base_Frame &frame, double x, double y, double z, double rx, double ry, double rz)
        : Pose_Base(x, y, z, rx, ry, rz), Frame_Bound(frame) {}

      Pose(const Base_Frame &frame, double x, double y, double z = 0.0)
        : Pose_Base(x, y, z, 0, 0, 0), Frame_Bound(frame) {}

      Pose() : Pose_Base(), Frame_Bound() {}

      Pose(const Location &loc)
        : Pose_Base(loc), Frame_Bound(loc.frame()) {}

      Pose(const Rotation &rot)
        : Pose_Base(rot), Frame_Bound(rot.frame()) {}

      Pose(const Base_Frame &frame, const Location_Base &loc, const Rotation_Base &rot)
        : Pose_Base(loc, rot), Frame_Bound(frame) {}

      /// Precondition: loc.frame == rot.frame
      Pose(const Location &loc, const Rotation &rot)
        : Pose_Base(loc, rot), Frame_Bound(loc.frame()) {}

      operator Location() const
      {
        return Location(frame(), x(), y(), z());
      }

      operator Rotation() const
      {
        return Rotation(frame(), rx(), ry(), rz());
      }

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }
    };
  }
}

// Include if not already included
#include <gams/utility/Base_Frame.h>

#endif
