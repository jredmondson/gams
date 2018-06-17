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
 * @file Coordinate.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Position, Orientation, and Pose classes
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_COORDINATE_H_
#define _GAMS_POSE_COORDINATE_H_

#include "gams/GamsExport.h"
#include <string>
#include <cfloat>
#include <utility>
#include <gams/CPP11_compat.h>
#include <madara/knowledge/containers/DoubleVector.h>
#include <madara/knowledge/containers/NativeDoubleVector.h>
#include "ReferenceFrameFwd.h"
#include <Eigen/Geometry>

#define INVAL_COORD DBL_MAX

namespace gams { namespace pose {

/// Type tags to indicate the dimension a coordinate lies within
namespace units {
  /// Tag that coordinate is a fixed vector, not free
  template<typename T>
  struct absolute {};

  struct length {};
  struct velocity {};
  struct acceleration {};
  struct plane_angle {};
  struct angular_velocity {};
  struct angular_acceleration {};

  /// Trait to strip absolute off a unit tag
  template<typename T>
  struct base_of
  {
    using type = T;
  };

  template<typename T>
  struct base_of<absolute<T>>
  {
    using type = T;
  };

  /// Trait to compare types, ignoring absolute
  template<typename L, typename R>
  struct same_base :
    std::is_same<typename base_of<L>::type,
                 typename base_of<R>::type> {};
}

/// Helper trait to customize BasicVector based on unit
template<typename Units>
struct unit_traits
{
  static_assert(sizeof(Units) < 0, "Unit type not supported by BasicVector");
};

/// Internal use
struct default_unit_traits
{
  class storage_mixin
  {
  private:
    Eigen::Vector3d vec_;

  public:
    storage_mixin() = default;

    storage_mixin(double x, double y)
      : vec_(x, y, 0) {}

    storage_mixin(double x, double y, double z)
      : vec_(x, y, z) {}

    explicit storage_mixin(const Eigen::Vector3d &vec) : vec_(vec) {}

    explicit storage_mixin(const madara::knowledge::containers::DoubleVector &v)
      : vec_(v.size() > 0 ? v[0] : 0,
             v.size() > 1 ? v[1] : 0,
             v.size() > 2 ? v[2] : 0) {}

    explicit storage_mixin(const madara::knowledge::containers::NativeDoubleVector &v)
      : vec_(v.size() > 0 ? v[0] : 0,
             v.size() > 1 ? v[1] : 0,
             v.size() > 2 ? v[2] : 0) {}

    Eigen::Vector3d &vec() { return vec_; }
    const Eigen::Vector3d &vec() const { return vec_; }
  };
};

/// Internal use
struct default_positional_unit_traits : default_unit_traits
{
  static const bool positional = true;
};

class Quaternion;

inline Eigen::Vector3d quat_to_axis_angle(const Eigen::Quaterniond &quat)
{
  double norm = sqrt(quat.x() * quat.x() +
                     quat.y() * quat.y() +
                     quat.z() * quat.z());
  double angle = 2 * atan2(norm, quat.w());
  double sin_half_angle = sin(angle / 2);
  if(sin_half_angle < 1e-10)
  {
    return {0, 0, 0};
  }
  return {(quat.x() / sin_half_angle) * angle,
          (quat.y() / sin_half_angle) * angle,
          (quat.z() / sin_half_angle) * angle};
}

/// Internal use
struct default_rotational_unit_traits : default_unit_traits
{
  static const bool positional = false;

  class storage_mixin : public default_unit_traits::storage_mixin
  {
  public:
    using Base = default_unit_traits::storage_mixin;
    using Base::Base;

    storage_mixin() = default;

    storage_mixin(const Quaternion &quat);

    template<typename Units>
    storage_mixin(double rx, double ry, double rz, Units units)
      : Base(units.to_radians(rx),
             units.to_radians(ry),
             units.to_radians(rz)) {}

  public:
    /**
     * Construct from a Quaternion
     * @param quat the Quaternion to build from
     **/
    storage_mixin(const Eigen::Quaterniond &quat)
      : Base(quat_to_axis_angle(quat)) {}
  };
};

/// Internal use
template<typename Derived>
struct basic_positional_mixin
{
  Derived &self() { return static_cast<Derived&>(*this); }
  const Derived &self() const { return static_cast<const Derived&>(*this); }

  double x() const { return self().vec()[0]; }
  double y() const { return self().vec()[1]; }
  double z() const { return self().vec()[2]; }

  double x(double v) { return (self().vec()[0] = v); }
  double y(double v) { return (self().vec()[1] = v); }
  double z(double v) { return (self().vec()[2] = v); }
};

template<>
struct unit_traits<units::absolute<units::length>>
  : default_positional_unit_traits
{
  static const bool free = false;

  template<typename Derived>
  struct mixin : basic_positional_mixin<Derived> {
    using Base = basic_positional_mixin<Derived>;
    using Base::x;
    using Base::y;
    using Base::z;
    using Base::self;

    double lng () const { return x(); }
    double lon () const { return x(); }
    double longitude () const { return x(); }

    double lat () const { return y(); }
    double latitude () const { return y(); }

    double alt () const { return -z(); }
    double altitude () const { return -z(); }

    double lng (double v) { return x(v); }
    double lon (double v) { return x(v); }
    double longitude (double v) { return x(v); }

    double lat (double v) { return y(v); }
    double latitude (double v) { return y(v); }

    double alt (double v) { return z(-v); }
    double altitude (double v) { return z(-v); }

    Eigen::Vector3d &pos_vec() { return self().vec(); }
    const Eigen::Vector3d &pos_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::length>
  : default_positional_unit_traits
{
  static const bool free = true;

  template<typename Derived>
  struct mixin : basic_positional_mixin<Derived> {
    using Base = basic_positional_mixin<Derived>;
    using Base::self;

    Eigen::Vector3d &dis_vec() { return self().vec(); }
    const Eigen::Vector3d &dis_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::velocity>
  : default_positional_unit_traits
{
  static const bool free = true;

  template<typename Derived>
  struct mixin {
    Derived &self() { return static_cast<Derived&>(*this); }
    const Derived &self() const { return static_cast<const Derived&>(*this); }

    double dx() const { return self().vec()[0]; }
    double dy() const { return self().vec()[1]; }
    double dz() const { return self().vec()[2]; }

    double dx(double v) { return (self().vec()[0] = v); }
    double dy(double v) { return (self().vec()[1] = v); }
    double dz(double v) { return (self().vec()[2] = v); }

    Eigen::Vector3d &vel_vec() { return self().vec(); }
    const Eigen::Vector3d &vel_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::acceleration>
  : default_positional_unit_traits
{
  static const bool positional = true;
  static const bool free = true;

  template<typename Derived>
  struct mixin {
    Derived &self() { return static_cast<Derived&>(*this); }
    const Derived &self() const { return static_cast<const Derived&>(*this); }

    double ddx() const { return self().vec()[0]; }
    double ddy() const { return self().vec()[1]; }
    double ddz() const { return self().vec()[2]; }

    double ddx(double v) { return (self().vec()[0] = v); }
    double ddy(double v) { return (self().vec()[1] = v); }
    double ddz(double v) { return (self().vec()[2] = v); }

    Eigen::Vector3d &acc_vec() { return self().vec(); }
    const Eigen::Vector3d &acc_vec() const { return self().vec(); }
  };
};

/// Internal use
template<typename Derived>
struct common_rotational_mixin
{
  Derived &self() { return static_cast<Derived&>(*this); }
  const Derived &self() const { return static_cast<const Derived&>(*this); }

  double angle_to(const Derived &target) const
  {
    return self().distance_to(target);
  }

  template<typename U>
  double angle_to (const Derived &target, U u) const
  {
    return u.from_radians (self().angle_to (target));
  }
};

/// Internal use
template<typename Derived>
struct basic_rotational_mixin : common_rotational_mixin<Derived>
{
  using Base = common_rotational_mixin<Derived>;
  using Base::self;

  double rx() const { return self().vec()[0]; }
  double ry() const { return self().vec()[1]; }
  double rz() const { return self().vec()[2]; }

  double rx(double v) { return (self().vec()[0] = v); }
  double ry(double v) { return (self().vec()[1] = v); }
  double rz(double v) { return (self().vec()[2] = v); }

  /**
   * Represent this rotation as a quaternion.
   * @return Quaternion representation.
   **/
  Eigen::Quaterniond into_quat() const
  {
    double magnitude = sqrt(rx() * rx() + ry() * ry() + rz() * rz());
    if(magnitude == 0)
    {
      return {1, 0, 0, 0};
    }
    double half_mag = magnitude / 2;
    double cos_half_mag = cos(half_mag);
    double sin_half_mag = sin(half_mag);
    return {cos_half_mag,
            (rx() / magnitude) * sin_half_mag,
            (ry() / magnitude) * sin_half_mag,
            (rz() / magnitude) * sin_half_mag};
  }

  void from_quat(const Eigen::Quaterniond &quat)
  {
    this->vec() = quat_to_axis_angle(quat);
  }

  template<typename Other>
  auto slerp(double scale, const Other &other) ->
    typename std::decay<decltype(other.into_quat(),
        std::declval<Derived>())>::type
  {
    return Derived(into_quat().slerp(scale, other.into_quat()));
  }
};

template<>
struct unit_traits<units::absolute<units::plane_angle>>
  : default_rotational_unit_traits
{
  static const bool free = false;

  template<typename Derived>
  struct mixin : basic_rotational_mixin<Derived> {
    using Base = basic_rotational_mixin<Derived>;
    using Base::self;

    Eigen::Vector3d &ori_vec() { return self().vec(); }
    const Eigen::Vector3d &ori_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::plane_angle>
  : default_rotational_unit_traits
{
  static const bool free = true;

  template<typename Derived>
  struct mixin : basic_rotational_mixin<Derived> {
    using Base = basic_rotational_mixin<Derived>;
    using Base::self;

    Eigen::Vector3d &rot_vec() { return self().vec(); }
    const Eigen::Vector3d &rot_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::angular_velocity>
  : default_rotational_unit_traits
{
  static const bool free = true;

  template<typename Derived>
  struct mixin {
    Derived &self() { return static_cast<Derived&>(*this); }
    const Derived &self() const { return static_cast<const Derived&>(*this); }

    double drx() const { return self().vec()[0]; }
    double dry() const { return self().vec()[1]; }
    double drz() const { return self().vec()[2]; }

    double drx(double v) { return (self().vec()[0] = v); }
    double dry(double v) { return (self().vec()[1] = v); }
    double drz(double v) { return (self().vec()[2] = v); }

    Eigen::Vector3d &ang_vel_vec() { return self().vec(); }
    const Eigen::Vector3d &ang_vel_vec() const { return self().vec(); }
  };
};

template<>
struct unit_traits<units::angular_acceleration>
  : default_rotational_unit_traits
{
  static const bool positional = true;
  static const bool free = true;

  template<typename Derived>
  struct mixin {
    Derived &self() { return static_cast<Derived&>(*this); }
    const Derived &self() const { return static_cast<const Derived&>(*this); }

    double ddrx() const { return self().vec()[0]; }
    double ddry() const { return self().vec()[1]; }
    double ddrz() const { return self().vec()[2]; }

    double ddrx(double v) { return (self().vec()[0] = v); }
    double ddry(double v) { return (self().vec()[1] = v); }
    double ddrz(double v) { return (self().vec()[2] = v); }

    Eigen::Vector3d &ang_acc_vec() { return self().vec(); }
    const Eigen::Vector3d &ang_acc_vec() const { return self().vec(); }
  };
};

/// For internal use. The underlying template for all coordinate types.
template<typename Derived, typename Units>
class BasicVector
  : public unit_traits<Units>::storage_mixin,
    public unit_traits<Units>::template mixin<Derived>
{
private:
  Eigen::Vector3d vec_;

  using traits = unit_traits<Units>;
  using storage_mixin = typename unit_traits<Units>::storage_mixin;
  using mixin = typename unit_traits<Units>::template mixin<Derived>;

public:
  using storage_mixin::storage_mixin;
  using storage_mixin::vec;

  using mixin::self;

  using derived_type = Derived;
  using units_type = Units;

  BasicVector() = default;

  template<typename Derived2>
  BasicVector(const BasicVector<Derived2, Units> &v) : storage_mixin(v.vec()) {}

  /// Is this coordinate a positional one?
  static constexpr bool positional() { return traits::positional; }

  /// Is this coordinate a rotational one?
  static constexpr bool rotational() { return !positional(); }

  /// Is this coordinate a free vector?
  static constexpr bool free() { return traits::free; }

  /// Is this coordinate a fixed vector?
  static constexpr bool fixed() { return !free(); }

  /// Get number of values in this coordinate
  size_t size() const { return (size_t)vec().size(); }

  /**
   * Get i'th value in this Coordinate. No range checking!
   **/
  double get(size_t i) const { return vec()[i]; }

  /**
   * Set i'th value in this Coordinate. No range checking!
   *
   * @param v the value to set to
   * @return the new value
   **/
  double set(size_t i, double v) { return (vec()[i] = v); }

  /// Does this coordinate have any values not INVAL_COORD?
  bool is_set() const
  {
    for (int i = 0; i < vec().size(); ++i) {
      if (vec()[i] != INVAL_COORD) {
        return true;
      }
    }
    return false;
  }

  /// Does this coordinate have values all zeroes?
  bool is_zero() const
  {
    for (int i = 0; i < vec().size(); ++i) {
      if (vec()[i] != 0) {
        return false;
      }
    }
    return true;
  }

  /**
   * Outputs this Coordinates values to the referenced container. This
   * container type must support operator[] for setting by index.
   *
   * If the array's size is smaller than the cardinality of this
   * coordinate type, the behavior is undefined. If it is larger, the
   * extra elements are not changed.
   *
   * The MADARA DoubleVector and NativeDoubleVector types are supported.
   *
   * @tparam ContainType the type of the container; must support "set"
   * @param out the container to put this Coordinate's values into.
   **/
  template<typename ContainType>
  void to_array(ContainType &out) const
  {
    for(int i = 0; i < size(); i++)
    {
      out[i] = get(i);
    }
  }

  /**
   * Overwrites this Coordinate's values with those pulled from the
   * referenced array. These values will be within this object's
   * current reference frame. The container must support operator[],
   *
   * If the array's size is smaller than the cardinality of this
   * coordinate type, the behavior is undefined. If it is larger, the
   * extra elements are ignored.
   *
   * @tparam ContainType the type of the container; must support operator[]
   * @param in the container to pull new values from.
   **/
  template<typename ContainType>
  void from_array(const ContainType &in)
  {
    for(int i = 0; i < size(); i++)
    {
      set(i, in[i]);
    }
  }

  /**
  * Returns a string of the values x, y, z
  * @param delimiter      delimiter between values
  * @param unset_identifier  value to print if unset
  * @return  stringified version of the Linear
  **/
  std::string to_string (
      const std::string & delimiter = ",",
      const std::string & unset_identifier = "<unset>") const
  {
    std::stringstream buffer;
    bool first = true;

    for (int i = 0; i < vec().size(); ++i)
    {
      auto x = vec()[i];

      if (!first) {
        buffer << delimiter;
      } else {
        first = false;
      }

      if (x != INVAL_COORD)
        buffer << x;
      else
        buffer << unset_identifier;
    }

    return buffer.str ();
  }

  void to_container (
    madara::knowledge::containers::NativeDoubleVector &container) const
  {
    container.set (0, this->get (0));
    container.set (1, this->get (1));
    container.set (2, this->get (2));
  }

  void from_container (
    const madara::knowledge::containers::NativeDoubleVector &container)
  {
    this->set (0, container[0]);
    this->set (1, container[1]);
    this->set (2, container[2]);
  }

  /**
   * Passthrough to Eigen vector dot method
   **/
  template<typename Other>
  double dot(const BasicVector<Other, Units> &other) const
  {
    return this->vec().dot(other.vec());
  }

  /**
   * Passthrough to Eigen vector cross method
   **/
  template<typename Other>
  Derived cross(const BasicVector<Other, Units> &other) const
  {
    Derived ret = this->self();
    ret.vec() = this->vec().cross(other.vec());
    return ret;
  }

  /**
   * Passthrough to Eigen vector norm method
   **/
  double norm() const
  {
    return this->vec().norm();
  }

  /**
   * Passthrough to Eigen vector squaredNorm method
   **/
  double squaredNorm() const
  {
    return this->vec().squaredNorm();
  }

  /**
   * Passthrough to Eigen vector normalized method
   **/
  Derived normalized() const
  {
    Derived ret = this->self();
    ret.vec().normalize();
    return ret;
  }

  /**
   * Tests if this Coordinate is within epsilon in distance (as defined by
   * this Coordinate's reference frame's distance metric). If the other
   * Coordinate is in a different reference frame, it is first copied, and
   * converted to this Coordinate's reference frame.
   *
   * @param other the other Coordinate to test against
   * @param epsilon the maximum distance permitted to return true
   * @return true if the distance is less than or equal to  epsilon
   **/
  /*template<typename Derived2>
  bool approximately_equal(const BasicVector<Derived2, Units> &other,
      double epsilon) const
  {
    return std::fabs(self().distance_to(other.self())) < epsilon;
  }*/

  /**
   * Calculate distance from this Coordinate to a target. If the target
   * is in another reference frame, this and the target will be copied, and
   * converted to their closest common frame.
   *
   * Requres "ReferenceFrame.h"
   *
   * @param target the target Coordinate to calculate distance to
   * @return the distance according to the distance metric in the common
   *   frame, for CoordType. Typically, return will be meters or degrees.
   *
   * @throws unrelated_frames thrown if the target's reference frame is not
   *      part of the same tree as the current one.
   * @throws undefined_transform thrown if no conversion between two frames
   *      along the conversion path has been defined.
   **/
  /*template<typename Derived2>
  double distance_to(const BasicVector<Derived2, Units> &target) const {
    return (target.vec() - vec()).norm();
  }*/
};

} }

#include "Coordinate.inl"

#include "Framed.h"
#include "Stamped.h"

namespace gams { namespace pose {

#define GAMS_POSE_MAKE_COMPOSITE_VEC_SCALAR_OP(op) \
template<typename Derived, typename Units, typename Scalar> \
inline auto operator op##=(BasicVector<Derived, Units> &vec, Scalar scalar) -> \
  typename std::enable_if<std::is_arithmetic<Scalar>::value && \
                          BasicVector<Derived, Units>::free(), \
                          typename BasicVector<Derived, Units>::derived_type&>::type \
{ \
  vec.vec() op##= scalar; \
  return vec.self(); \
}

#define GAMS_POSE_MAKE_BINARY_VEC_SCALAR_OP(op) \
template<typename Derived, typename Units, typename Scalar> \
inline auto operator op(const BasicVector<Derived, Units> &vec, Scalar scalar) -> \
  typename std::enable_if<std::is_arithmetic<Scalar>::value && \
                          BasicVector<Derived, Units>::free(), \
                          typename BasicVector<Derived, Units>::derived_type>::type \
{ \
  typename BasicVector<Derived, Units>::derived_type ret = vec.self(); \
  ret op##= scalar; \
  return ret; \
} \
\
template<typename Derived, typename Units, typename Scalar> \
inline auto operator op(Scalar scalar, const BasicVector<Derived, Units> &vec) -> \
  decltype(vec op scalar) \
{ \
  return vec op scalar; \
}

#define GAMS_POSE_MAKE_VEC_SCALAR_OPS(op) \
GAMS_POSE_MAKE_COMPOSITE_VEC_SCALAR_OP(op) \
GAMS_POSE_MAKE_BINARY_VEC_SCALAR_OP(op)

GAMS_POSE_MAKE_VEC_SCALAR_OPS(*)
GAMS_POSE_MAKE_VEC_SCALAR_OPS(/)

#define GAMS_POSE_MAKE_COMPOSITE_VECS_OP(op) \
template<typename LDerived, \
         typename RDerived, typename Units> \
inline auto operator op##=(BasicVector<LDerived, Units> &lhs, \
              const Eigen::MatrixBase<RDerived> &rhs) -> \
  typename std::enable_if<unit_traits<Units>::positional, \
                          LDerived &>::type \
{ \
  lhs.vec() op##= rhs; \
  return lhs.self(); \
} \
\
template<typename LDerived, \
         typename RDerived, typename Units> \
inline auto operator op##=(BasicVector<LDerived, Units> &lhs, \
                 const BasicVector<RDerived, Units> &rhs) -> \
  typename std::enable_if<unit_traits<Units>::positional && \
                          unit_traits<Units>::free, LDerived &>::type \
{ \
  lhs.vec() op##= rhs.vec(); \
  return lhs.self(); \
} \
\
template<typename LDerived, \
         typename RDerived, typename Units> \
inline auto operator op##=(BasicVector<LDerived, units::absolute<Units>> &lhs, \
                 const BasicVector<RDerived, Units> &rhs) -> \
  typename std::enable_if<unit_traits<Units>::positional && \
                          unit_traits<Units>::free, LDerived &>::type \
{ \
  lhs.vec() op##= rhs.vec(); \
  return lhs.self(); \
} \

GAMS_POSE_MAKE_COMPOSITE_VECS_OP(+)
GAMS_POSE_MAKE_COMPOSITE_VECS_OP(-)

template<typename LDerived, typename LUnits,
         typename RDerived, typename RUnits>
inline auto operator+(const BasicVector<LDerived, LUnits> &lhs,
               const BasicVector<RDerived, RUnits> &rhs) ->
  typename std::enable_if<RDerived::free(), LDerived>::type
{
  LDerived ret = lhs.self();
  ret += rhs;
  return ret;
}

template<typename LDerived, typename LUnits,
         typename RDerived, typename RUnits>
inline auto operator+(const BasicVector<LDerived, LUnits> &lhs,
               const BasicVector<RDerived, RUnits> &rhs) ->
  typename std::enable_if<LDerived::free() && RDerived::fixed(), RDerived>::type
{
  RDerived ret = rhs.self();
  ret += lhs;
  return ret;
}

template<typename LDerived, typename LUnits,
         typename RDerived, typename RUnits>
inline auto operator-(const BasicVector<LDerived, LUnits> &lhs,
               const BasicVector<RDerived, RUnits> &rhs) ->
  typename std::enable_if<LDerived::free() && RDerived::free(), LDerived>::type
{
  LDerived ret = lhs.self();
  ret -= rhs;
  return ret;
}

/// Helper struct for defining which fixed coordinate types map to
/// which free vector coordinate types. Add specializations as needed.
template<typename T>
struct fixed_into_free {};

template<typename LDerived, typename LUnits,
         typename RDerived, typename RUnits>
inline auto operator-(const BasicVector<LDerived, LUnits> &lhs,
               const BasicVector<RDerived, RUnits> &rhs) ->
  typename std::enable_if<LDerived::fixed() && RDerived::fixed(),
    typename fixed_into_free<LDerived>::type>::type
{
  typename fixed_into_free<LDerived>::type ret(lhs);
  ret.vec() -= rhs.vec();
  return ret;
}

#define GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(op) \
  template<typename LDerived, typename RDerived, typename Units> \
  inline bool operator op ( \
      const BasicVector<LDerived, Units> &lhs, \
      const BasicVector<RDerived, Units> &rhs) \
  { \
    for (int i = 0; i < lhs.vec().size(); ++i) { \
      if (!(lhs.vec()[i] op rhs.vec()[i])) { \
        return false; \
      } \
    } \
    return true; \
  } \
 \
  template<typename LDerived, typename RDerived, typename Units> \
  inline bool operator op ( \
      const Framed<BasicVector<LDerived, Units>> &lhs, \
      const Framed<BasicVector<RDerived, Units>> &rhs) \
  { \
    return lhs.frame() op rhs.frame() && \
      static_cast<const BasicVector<LDerived, Units> &>(lhs) op \
      static_cast<const BasicVector<RDerived, Units> &>(rhs); \
  } \
 \
  template<typename LDerived, typename RDerived, typename Units> \
  inline bool operator op ( \
      const Stamped<Framed<BasicVector<LDerived, Units>>> &lhs, \
      const Stamped<Framed<BasicVector<RDerived, Units>>> &rhs) \
  { \
    return lhs.time() op rhs.time() && \
      static_cast<const Framed<BasicVector<LDerived, Units> &>>(lhs) op \
      static_cast<const Framed<BasicVector<RDerived, Units> &>>(rhs); \
  }

GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(==)
GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(!=)
GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(<)
GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(<=)
GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(>)
GAMS_POSE_MAKE_COORDINATE_COMPARE_OPS(>=)

#define GAMS_POSE_MAKE_COORDINATE_TYPE(name, units) \
class name##Vector : \
  public BasicVector<name##Vector, units> \
{ \
public: \
  using Base = BasicVector<name##Vector, units>; \
  using Base::Base; \
  static constexpr const char * type_name = #name "Vector"; \
}; \
\
class name : \
  public Framed<BasicVector<name, units>> \
{ \
public: \
  using Base = Framed<BasicVector<name, units>>; \
  using Base::Base; \
  static constexpr const char * type_name = #name; \
  operator name##Vector () const { return name##Vector(this->vec()); } \
  name(name##Vector o) : Base(o.vec()) {} \
  name() = default; \
}; \
\
class Stamped##name : \
  public Stamped<Framed<BasicVector<Stamped##name, units>>> \
{ \
public: \
  using Base = Stamped<Framed<BasicVector<Stamped##name, units>>>; \
  using Base::Base; \
  static constexpr const char * type_name = "Stamped" #name; \
  operator name##Vector () const { return name##Vector(this->vec()); } \
  operator name () const { return name(this->frame(), this->vec()); } \
  Stamped##name(name##Vector o) : Base(o.vec()) {} \
  Stamped##name(name o) : Base(o.frame(), o.vec()) {} \
  Stamped##name() = default; \
}

/**
 * Class for fixed-vector positions in a reference frame
 **/
class Position;

/**
 * Class for free-vector displacements (i.e., differences between Positions)
 * in a reference frame
 **/
class Position;

/**
 * Class for velocities (free-vector) in a reference frame
 **/
class Velocity;

/**
 * Class for accelerations (free-vector) in a reference frame
 **/
class Accelerations;

/**
 * Class for fixed-vector orientations relative to frame basis
 * in a given ReferenceFrame
 *
 * Represents orientation in axis-angle form
 **/
class Orientation;

/**
 * Class for free-vector rotations (i.e., differences between Orientations)
 * in a given ReferenceFrame
 *
 * Represents orientation in axis-angle form
 **/
class Rotation;

/**
 * Class for angular velocities (free-vector) in a given ReferenceFrame
 *
 * Represents orientation in axis-angle form
 **/
class AngularVelocity;

/**
 * Class for angular accelerations (free-vector) in a given ReferenceFrame
 *
 * Represents orientation in axis-angle form
 **/
class AngularAccelerations;

GAMS_POSE_MAKE_COORDINATE_TYPE(Position, units::absolute<units::length>);
GAMS_POSE_MAKE_COORDINATE_TYPE(Displacement, units::length);
GAMS_POSE_MAKE_COORDINATE_TYPE(Velocity, units::velocity);
GAMS_POSE_MAKE_COORDINATE_TYPE(Accleration, units::acceleration);

GAMS_POSE_MAKE_COORDINATE_TYPE(Orientation, units::absolute<units::plane_angle>);
GAMS_POSE_MAKE_COORDINATE_TYPE(Rotation, units::plane_angle);
GAMS_POSE_MAKE_COORDINATE_TYPE(AngularVelocity, units::angular_velocity);
GAMS_POSE_MAKE_COORDINATE_TYPE(AngularAccleration, units::angular_acceleration);

template<>
struct fixed_into_free<PositionVector>
{
  using type = DisplacementVector;
};

template<>
struct fixed_into_free<Position>
{
  using type = Displacement;
};

template<>
struct fixed_into_free<StampedPosition>
{
  using type = StampedDisplacement;
};

template<>
struct fixed_into_free<OrientationVector>
{
  using type = RotationVector;
};

template<>
struct fixed_into_free<Orientation>
{
  using type = Rotation;
};

template<>
struct fixed_into_free<StampedOrientation>
{
  using type = StampedRotation;
};

inline std::ostream &operator<<(std::ostream &o, const Eigen::Vector3d &v)
{
  bool first = true;
  for (int i = 0; i < v.size(); ++i)
  {
    auto x = v[i];

    if (!first) {
      o << ",";
    } else {
      first = false;
    }

    if (x != INVAL_COORD) {
      o << x;
    } else {
      o << "<unset>";
    }
  }
  return o;
}

template<typename Derived, typename Units>
inline std::ostream &operator<<(std::ostream &o, const BasicVector<Derived, Units> &v)
{
  o << Derived::type_name << "(";
  o << v.vec();
  o << ")";
  return o;
}

} }

#endif
