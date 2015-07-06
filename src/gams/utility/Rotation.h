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

#ifndef _GAMS_UTILITY_ROTATION_H_
#define _GAMS_UTILITY_ROTATION_H_

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

#define DEG_TO_RAD(x) (((x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) (((x) * 180) / M_PI)

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    class Quaternion;

    /**
     * Container for Rotation information, not bound to a frame.
     **/
    class Rotation_Vector
    {
    public:
      static const int X_axis = 0;
      static const int Y_axis = 1;
      static const int Z_axis = 2;

      Rotation_Vector(double rx, double ry, double rz)
        : rx_(rx), ry_(ry), rz_(rz) {}

      Rotation_Vector(double x, double y, double z, double angle)
        : rx_(x * angle), ry_(y * angle), rz_(z * angle) {}

      Rotation_Vector(int axis_index, double angle)
        : rx_(axis_index == X_axis ? DEG_TO_RAD(angle) : 0),
          ry_(axis_index == Y_axis ? DEG_TO_RAD(angle) : 0),
          rz_(axis_index == Z_axis ? DEG_TO_RAD(angle) : 0) {}

      Rotation_Vector()
        : rx_(INVAL_COORD), ry_(INVAL_COORD), rz_(INVAL_COORD) {}

      Rotation_Vector(const Rotation_Vector &orig)
        : rx_(orig.rx_), ry_(orig.ry_), rz_(orig.rz_) {}

      explicit Rotation_Vector(const Quaternion &quat);

      bool is_invalid() const
      {
        return rx_ == INVAL_COORD || ry_ == INVAL_COORD || rz_ == INVAL_COORD;
      }

      bool is_zero() const
      {
        return rx_ == 0 && ry_ == 0 && rz_ == 0;
      }

      bool operator==(const Rotation_Vector &other) const
      {
        return rx_ == other.rx_ && ry_ == other.ry_ && rz_ == other.rz_;
      }

      static std::string name()
      {
        return "Rotation";
      }

      double rx() const { return rx_; }
      double ry() const { return ry_; }
      double rz() const { return rz_; }

      double rx(double new_rx) { return rx_ = new_rx; }
      double ry(double new_ry) { return ry_ = new_ry; }
      double rz(double new_rz) { return rz_ = new_rz; }

      typedef Rotation_Vector Base_Type;

      Base_Type &as_vec()
      {
        return static_cast<Base_Type &>(*this);
      }

      const Base_Type &as_vec() const
      {
        return static_cast<const Base_Type &>(*this);
      }

      int size() const
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
      double rx_, ry_, rz_;
    };

    inline std::ostream &operator<<(std::ostream &o, const Rotation_Vector &rot)
    {
      o << "(" << rot.rx() << "," << rot.ry() << "," << rot.rz() << ")";
      return o;
    }

    /**
     * Represents a rotation or orientation within a reference frame.
     *
     * All rotations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the rotation axis, rotations curve in the direction your
     * fingers are pointing.
     **/
    class Rotation : public Rotation_Vector, public Coordinate<Rotation>
    {
    public:
      /**
       * The direction of rx, ry, and rz is the axis of rotation, with respect
       * to the owning frame's axes.
       * The magnitude of rx, ry, and rz (as a vector) is the amount of rotation,
       * in radians.
       **/
      Rotation(double rx, double ry, double rz)
        : Rotation_Vector(rx, ry, rz), Coordinate() {}

      Rotation(const Reference_Frame &frame, double rx, double ry, double rz)
        : Rotation_Vector(rx, ry, rz), Coordinate(frame) {}

      /**
       * (x, y, z) must be a unit vector in the direction of rotation axis.
       * angle is the angle of rotation, in degrees, about that axis
       **/
      Rotation(double x, double y, double z, double angle)
        : Rotation_Vector(x, y, z, DEG_TO_RAD(angle)), Coordinate() {}

      Rotation(const Reference_Frame &frame, double x, double y, double z, double angle)
        : Rotation_Vector(x, y, z, DEG_TO_RAD(angle)), Coordinate(frame) {}

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       **/
      Rotation(int axis_index, double angle)
        : Rotation_Vector(axis_index, angle), Coordinate() {}

      Rotation(const Reference_Frame &frame, int axis_index, double angle)
        : Rotation_Vector(axis_index, angle), Coordinate(frame) {}

      Rotation() : Rotation_Vector(), Coordinate() {}

      Rotation(const Rotation &orig)
        : Rotation_Vector(orig), Coordinate(orig) {}

      explicit Rotation(const Quaternion &quat)
        : Rotation_Vector(quat) {}

      Rotation(const Reference_Frame &new_frame, const Rotation &orig)
        : Rotation_Vector(orig), Coordinate(orig.frame())
      {
        transform_this_to(new_frame);
      }

      double angle_to(const Rotation &target) const
      {
        return distance_to(target);
      }

      using Coordinate<Rotation>::operator==;
    };
  }
}

#include <gams/utility/Location.h>

namespace gams
{
  namespace utility
  {
    /**
     * Used internally to implement angle operations.
     * Not reference-frame aware.
     **/
    class Quaternion
    {
    public:
      Quaternion(double x, double y, double z, double w)
        : x_(x), y_(y), z_(z), w_(w) {}

      Quaternion(double rx, double ry, double rz)
      {
        from_rotation_vector(rx, ry, rz);
      }

      explicit Quaternion(const Rotation_Vector &rot)
      {
        from_rotation_vector(rot);
      }

      explicit Quaternion(const Location_Vector &loc)
      {
        from_location_vector(loc);
      }

      void from_location_vector(double x, double y, double z)
      {
        x_ = x;
        y_ = y;
        z_ = z;
        w_ = 0;
      }

      void from_location_vector(const Location_Vector &loc)
      {
        from_location_vector(loc.x(), loc.y(), loc.z());
      }

      void to_location_vector(double &x, double &y, double &z) const
      {
        x = x_;
        y = y_;
        z = z_;
      }

      void to_location_vector(Location_Vector &loc) const
      {
        to_location_vector(loc.x_, loc.y_, loc.z_);
      }

      void from_rotation_vector(double rx, double ry, double rz)
      {
        double magnitude = sqrt(rx * rx + ry * ry + rz * rz);
        if(magnitude == 0)
        {
          w_ = 1;
          x_ = y_ = z_ = 0;
        }
        else
        {
          double half_mag = magnitude / 2;
          double cos_half_mag = cos(half_mag);
          double sin_half_mag = sin(half_mag);
          w_ = cos_half_mag;
          x_ = (rx / magnitude) * sin_half_mag;
          y_ = (ry / magnitude) * sin_half_mag;
          z_ = (rz / magnitude) * sin_half_mag;
        }
      }

      void from_rotation_vector(const Rotation_Vector &rot)
      {
        from_rotation_vector(rot.rx(), rot.ry(), rot.rz());
      }

      void to_rotation_vector(double &rx, double &ry, double &rz) const
      {
        double norm = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        double angle = 2 * atan2(norm, w_);
        double sin_half_angle = sin(angle / 2);
        if(sin_half_angle < 1e-10)
        {
          rx = ry = rz = 0;
        }
        else
        {
          rx = (x_ / sin_half_angle) * angle;
          ry = (y_ / sin_half_angle) * angle;
          rz = (z_ / sin_half_angle) * angle;
        }
      }

      void to_rotation_vector(Rotation_Vector &rot) const
      {
        to_rotation_vector(rot.rx_, rot.ry_, rot.rz_);
      }

      static void hamilton_product(Quaternion &into, const Quaternion &lhs, const Quaternion &rhs)
      {
        double A = (lhs.w_ + lhs.x_) * (rhs.w_ + rhs.x_),
               B = (lhs.z_ - lhs.y_) * (rhs.y_ - rhs.z_),
               C = (lhs.w_ - lhs.x_) * (rhs.y_ + rhs.z_),
               D = (lhs.y_ + lhs.z_) * (rhs.w_ - rhs.x_),
               E = (lhs.x_ + lhs.z_) * (rhs.x_ + rhs.y_),
               F = (lhs.x_ - lhs.z_) * (rhs.x_ - rhs.y_),
               G = (lhs.w_ + lhs.y_) * (rhs.w_ - rhs.z_),
               H = (lhs.w_ - lhs.y_) * (rhs.w_ + rhs.z_);

        into.w_ = B + (-E - F + G + H) / 2;
        into.x_ = A - ( E + F + G + H) / 2;
        into.y_ = C + ( E - F + G - H) / 2;
        into.z_ = D + ( E - F - G + H) / 2;
      }

      Quaternion &operator*=(const Quaternion &rhs)
      {
        hamilton_product(*this, *this, rhs);

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
        x_ = -x_;
        y_ = -y_;
        z_ = -z_;
        return *this;
      }

      Quaternion operator-() const
      {
        Quaternion ret(-x_, -y_, -z_, w_);
        return ret;
      }

      double inner_product(const Quaternion &o) const
      {
        return w_ * o.w_ + x_ * o.x_ + y_ * o.y_ + z_ * o.z_;
      }

      double angle_to(const Quaternion &o) const
      {
        double prod = inner_product(o);
        return acos(2 * prod * prod - 1);
      }

      double x() const { return x_; }
      double y() const { return y_; }
      double z() const { return z_; }
      double w() const { return w_; }

      double x(double new_x) { return x_ = new_x; }
      double y(double new_y) { return y_ = new_y; }
      double z(double new_z) { return z_ = new_z; }
      double w(double new_w) { return w_ = new_w; }

    private:
      double x_, y_, z_, w_;
    };

    inline Rotation_Vector::Rotation_Vector(const Quaternion &quat)
    {
      quat.to_rotation_vector(rx_, ry_, rz_);
    }

    inline std::ostream &operator<<(std::ostream &o, const Quaternion &quat)
    {
      o << quat.w() << "+" << quat.x() << "i+" << quat.y() << "j+" << quat.z() << "z";
      return o;
    }
  }
}

// Include if not already included
#include <gams/utility/Pose.h>

#endif
