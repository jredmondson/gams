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
#include <vector>
#include <typeinfo>
#include <stdexcept>
#include <cmath>
#include <cfloat>
#include <climits>

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
     *      -- Have this function: static std::string get_name()
     **/
    template<typename CoordType>
    class GAMS_Export Frame_Bound
    {
    public:

      Frame_Bound() : frame(__INTERNAL__::default_frame) {}

      Frame_Bound(const Base_Frame &frame) : frame(&frame) {}

      Frame_Bound(const Base_Frame *frame) : frame(frame) {}

      Frame_Bound(const Frame_Bound &orig) : frame(orig.frame) {}

    protected:
      const Base_Frame *frame;

    public:
      const Base_Frame &get_frame() const { return *frame; }
      void set_frame(const Base_Frame &new_frame) { frame = &new_frame; }

      // Defined in Base_Frame.h
      CoordType transform_to(const Base_Frame &new_frame) const;
      void transform_this_to(const Base_Frame &new_frame);
      double distance_to(const Frame_Bound<CoordType> &target) const;
      void normalize();

      bool operator==(const Frame_Bound<CoordType> &other) const
      {
        if(*frame == *other.frame)
        {
          return static_cast<const typename CoordType::Base_Type &>(
                   static_cast<const CoordType &>(*this)) ==
                 static_cast<const typename CoordType::Base_Type &>(
                   static_cast<const CoordType &>(other));
        }
        else
        {
          CoordType tmp(*frame, static_cast<const CoordType &>(other));
          return static_cast<const typename CoordType::Base_Type &>(
                   static_cast<const CoordType &>(*this)) ==
                 static_cast<const typename CoordType::Base_Type &>(
                   static_cast<const CoordType &>(tmp));
        }
      }

      bool operator!=(const Frame_Bound<CoordType> &other) const
      {
        return !(*this == other);
      }

      friend class Base_Frame;
    };

    /**
     * Container for Location information, not bound to a frame.
     * Do not use unless implementing new frames or coordinate types.
     **/
    class GAMS_Export Location_Base
    {
    public:
      Location_Base(double x, double y, double z = 0.0)
        : x(x), y(y), z(z) {}

      Location_Base()
        : x(INVAL_COORD), y(INVAL_COORD), z(INVAL_COORD) {}

      Location_Base(const Location_Base &orig)
        : x(orig.x), y(orig.y), z(orig.z) { }

      bool isInvalid()
      {
        return x == INVAL_COORD || y == INVAL_COORD || z == INVAL_COORD;
      }

      bool operator==(const Location_Base &other) const
      {
        return x == other.x && y == other.y && z == other.z;
      }

      double x, y, z;

      static std::string get_name()
      {
        return "Location";
      }

      typedef Location_Base Base_Type;
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
        : Location_Base(orig), Frame_Bound(orig.get_frame())
      {
        transform_this_to(new_frame);
        set_frame(new_frame);
      }

      using Frame_Bound<Location>::operator==;
    };

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
        : rx(rx), ry(ry), rz(rz) {}

      /**
       * (x, y, z) must be a unit vector in the direction of rotation axis.
       * angle is the angle of rotation about that axis
       **/
      Rotation_Base(double x, double y, double z, double angle)
        : rx(x * angle), ry(y * angle), rz(z * angle) {}

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle
       **/
      Rotation_Base(int axis_index, double angle)
        : rx(axis_index == X_axis ? DEG_TO_RAD(angle) : 0),
          ry(axis_index == Y_axis ? DEG_TO_RAD(angle) : 0),
          rz(axis_index == Z_axis ? DEG_TO_RAD(angle) : 0) {}

      Rotation_Base()
        : rx(INVAL_COORD), ry(INVAL_COORD), rz(INVAL_COORD) {}

      Rotation_Base(const Rotation_Base &orig)
        : rx(orig.rx), ry(orig.ry), rz(orig.rz) {}

      bool isInvalid() {
        return rx == INVAL_COORD || ry == INVAL_COORD || rz == INVAL_COORD;
      }

      double rx, ry, rz;

      static std::string get_name()
      {
        return "Rotation";
      }

      typedef Rotation_Base Base_Type;
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

      Rotation(const Rotation & orig)
        : Rotation_Base(orig), Frame_Bound(orig) {}
    };

    /**
     * Used internally to implement angle operations.
     * Not reference-frame aware.
     **/
    class GAMS_Export Quaternion
    {
    public:
      Quaternion(double x, double y, double z, double w)
        : x(x), y(y), z(z), w(w) {}

      Quaternion(double rx, double ry, double rz)
      {
        from_rotation_vector(rx, ry, rz);
      }

      Quaternion(const Rotation_Base &rot)
      {
        from_rotation_vector(rot);
      }

      void from_rotation_vector(double rx, double ry, double rz)
      {
        double magnitude = sqrt(rx * rx + ry * ry + rz * rz);
        if(magnitude == 0)
        {
          w = 1;
          x = y = z = 0;
        }
        else
        {
          double half_mag = magnitude / 2;
          double cos_half_mag = cos(half_mag);
          double sin_half_mag = sin(half_mag);
          w = cos_half_mag;
          x = (rx / magnitude) * sin_half_mag;
          y = (ry / magnitude) * sin_half_mag;
          z = (rz / magnitude) * sin_half_mag;
        }
      }

      void from_rotation_vector(const Rotation_Base &rot)
      {
        from_rotation_vector(rot.rx, rot.ry, rot.rz);
      }

      void to_rotation_vector(double &rx, double &ry, double &rz)
      {
        double norm = sqrt(x * x + y * y + z * z);
        double angle = 2 * atan2(norm, w);
        double sin_half_angle = sin(angle / 2);
        if(angle == 0)
        {
          rx = ry = rz = 0;
        }
        else
        {
          rx = (x / sin_half_angle) * angle;
          ry = (y / sin_half_angle) * angle;
          rz = (z / sin_half_angle) * angle;
        }
      }

      void to_rotation_vector(Rotation_Base &rot)
      {
        to_rotation_vector(rot.rx, rot.ry, rot.rz);
      }

      Quaternion &operator*=(const Quaternion &o)
      {
        double A = (w + x) * (o.w + o.x),
               B = (z - y) * (o.y - o.z),
               C = (w - x) * (o.y + o.z), 
               D = (y + z) * (o.w - o.x),
               E = (x + z) * (o.x + o.y),
               F = (x - z) * (o.x - o.y),
               G = (w + y) * (o.w - o.z),
               H = (w - y) * (o.w + o.z);

        w = B + (-E - F + G + H) / 2;
        x = A - ( E + F + G + H) / 2; 
        y = C + ( E - F + G - H) / 2; 
        z = D + ( E - F - G + H) / 2;

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
        x = -x;
        y = -y;
        z = -z;
        return *this;
      }

      Quaternion operator-() const
      {
        Quaternion ret(-x, -y, -z, w);
        return ret;
      }

      double inner_product(const Quaternion &o) const
      {
        return w * o.w + x * o.x + y * o.y + z * o.z;
      }

      double angle_to(const Quaternion &o) const
      {
        double prod = inner_product(o);
        return acos(2 * prod * prod - 1);
      }

      double x, y, z, w;
    };

    inline std::ostream &operator<<(std::ostream &o, const Quaternion &quat)
    {
      o << quat.w << "+" << quat.x << "i+" << quat.y << "j+" << quat.z << "z";
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

      static std::string get_name()
      {
        return "Pose";
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
        : Pose_Base(loc), Frame_Bound(loc.get_frame()) {}

      Pose(const Rotation &rot)
        : Pose_Base(rot), Frame_Bound(rot.get_frame()) {}

      Pose(const Base_Frame &frame, const Location_Base &loc, const Rotation_Base &rot)
        : Pose_Base(loc, rot), Frame_Bound(frame) {}

      /// Precondition: loc.frame == rot.frame
      Pose(const Location &loc, const Rotation &rot)
        : Pose_Base(loc, rot), Frame_Bound(loc.get_frame()) {}

      operator Location() const
      {
        return Location(*frame, x, y, z);
      }

      operator Rotation() const
      {
        return Rotation(*frame, rx, ry, rz);
      }

      bool isInvalid() {
        return Location_Base::isInvalid() || Rotation_Base::isInvalid();
      }
    };

    template<typename CoordType>
    class GAMS_Export Derivative
    {
    public:
      Derivative() : d() {}

      template<typename T1>
      Derivative(const T1 &p1) : d(p1) {}

      template<typename T1, typename T2>
      Derivative(const T1 &p1,
                 const T2 &p2)
        : d(p1, p2) {}

      template<typename T1, typename T2, typename T3>
      Derivative(const T1 &p1, const T2 &p2,
                 const T3 &p3)
        : d(p1, p2, p3) {}

      template<typename T1, typename T2, typename T3, typename T4>
      Derivative(const T1 &p1, const T2 &p2, const T3 &p3,
                 const T4 &p4)
        : d(p1, p2, p3, p4) {}

      template<typename T1, typename T2, typename T3, typename T4, typename T5>
      Derivative(const T1 &p1, const T2 &p2, const T3 &p3,
                 const T4 &p4, const T5 &p5)
        : d(p1, p2, p3, p4, p5) {}

      template<typename T1, typename T2, typename T3, typename T4,
               typename T5, typename T6>
      Derivative(const T1 &p1, const T2 &p2, const T3 &p3,
                 const T4 &p4, const T5 &p5, const T6 &p6)
        : d(p1, p2, p3, p4, p5, p6) {}

      template<typename T1, typename T2, typename T3, typename T4,
               typename T5, typename T6, typename T7>
      Derivative(const T1 &p1, const T2 &p2, const T3 &p3,
                 const T4 &p4, const T5 &p5, const T6 &p6,
                 const T7 &p7)
        : d(p1, p2, p3, p4, p5, p6, p7) {}

      CoordType d;

      static std::string get_name()
      {
        return "d" + CoordType::get_name();
      }

      typedef CoordType Base_Type;
    };

    typedef Derivative<Location> Velocity;
    typedef Derivative<Rotation> AngularVelocity;
    typedef Derivative<Velocity> Acceleration;
    typedef Derivative<AngularVelocity> AngularAcceleration;
  }
}

// Include if not already included
#include <gams/utility/Base_Frame.h>

#endif
