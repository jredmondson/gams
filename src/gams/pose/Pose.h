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
 *      are those of the author (s) and do not necessarily reflect the views of
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
 * @file Pose.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Pose class
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_POSE_H_
#define _GAMS_POSE_POSE_H_

#include <iostream>
#include <string>

#include <gams/pose/Position.h>
#include <gams/pose/Orientation.h>

namespace gams { namespace pose {

/**
 * Underlying template for all Pose types, including PoseVector,
 * Pose, and StampedPose
 * See PositionVector and OrientationVector for representation deatils
 **/
template<typename Derived>
class BasicPose : public PositionVector, public OrientationVector
{
public:
  using derived_type = Derived;

  /**
   * Constructs a BasicPose from individual position values
   *
   * @param x position along x-axis
   * @param y position along y-axis
   **/
  BasicPose (double x, double y)
    : BasicPose(x, y, 0, 0, 0, 0) {}

  /**
   * Constructs a BasicPose from individual position values
   *
   * @param x position along x-axis
   * @param y position along y-axis
   * @param z position along z-axis
   **/
  BasicPose (double x, double y, double z)
    : BasicPose(x, y, z, 0, 0, 0) {}


  /**
   * Constructs a BasicPose from individual position and orientation values
   *
   * @param x position along x-axis
   * @param y position along y-axis
   * @param z position along z-axis
   * @param rx length of orientation vector along x-axis
   * @param ry length of orientation vector along y-axis
   * @param rz length of orientation vector along z-axis
   **/
  BasicPose (double x, double y, double z,
              double rx, double ry, double rz);

  /**
   * Construct a BasicPose from a PositionVector. Orientation info will
   * be all zeros (is_orientation_zero () == true)
   *
   * @param loc the PositionVector to get position info from.
   **/
  BasicPose (const PositionVector &pos);

  /**
   * Construct a BasicPose from a OrientationVector. Position info will
   * be all zeros (is_position () == true)
   *
   * @param rot the OrientationVector to get position info from.
   **/
  BasicPose (const OrientationVector &rot);

  /**
   * Construct from individual PositionVector and OrientationVector
   *
   * @param loc the PositionVector
   * @param rot the OrientationVector
   **/
  BasicPose (const PositionVector &pos,
             const OrientationVector &rot);

  /**
   * Default constructor. All values will be INVAL_COORD
   **/
  BasicPose ();

  Derived &self() { return static_cast<Derived&>(*this); }
  const Derived &self() const { return static_cast<const Derived&>(*this); }

  template<typename Derived2>
  operator BasicVector<Derived2, units::absolute<units::length>>() const
  {
    return as_position_vec();
  }

  template<typename Derived2>
  operator BasicVector<Derived2, units::absolute<units::plane_angle>>() const
  {
    return as_orientation_vec();
  }

  /**
   * Constructor from MADARA DoubleVector
   * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
   *            to x, y, z, rx, ry, rz)
   **/
  explicit BasicPose (
    const madara::knowledge::containers::DoubleVector &vec);

  /**
   * Constructor from MADARA NativeDoubleVector
   * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
   *            to x, y, z, rx, ry, rz)
   **/
  explicit BasicPose (
    const madara::knowledge::containers::NativeDoubleVector &vec);

  /**
   * Constructor from two MADARA DoubleVectors, for position and orientation
   * @param vec_loc  position values from (0, 1, 2 into x, y, z)
   * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
   **/
  explicit BasicPose (
    const madara::knowledge::containers::DoubleVector &vec_loc,
    const madara::knowledge::containers::DoubleVector &vec_rot);

  /**
   * Constructor from two MADARA NativeDoubleVector, for position/orientation
   * @param vec_loc  position values from (0, 1, 2 into x, y, z)
   * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
   **/
  explicit BasicPose (
    const madara::knowledge::containers::NativeDoubleVector &vec_loc,
    const madara::knowledge::containers::NativeDoubleVector &vec_rot);

  /**
   * Tests if this Pose is invalid; i.e., any values are INVAL_COORD
   *
   * @return true if at least one value is INVAL_COORD
   **/
  bool is_set () const;

  /**
  * Tests if the position is set (valid).
  *
  * @return true if position has been set to something valid
  **/
  bool is_position_set () const;
  bool is_location_set () const;

  /**
  * Tests if the orientation/orientation has been set
  *
  * @return true if orientation has been set to something valid
  **/
  bool is_orientation_set () const;

  /**
   * Tests if all position information is zero.
   *
   * @return true if all position information is zero
   **/
  bool is_position_zero () const;
  bool is_location_zero () const;

  /**
   * Tests if all orientation information is zero.
   *
   * @return true if all orientation information is zero
   **/
  bool is_orientation_zero () const;

  /**
   * Tests if all pose information is zero.
   * If true, is_position_zero and is_orientation_zero are also true
   *
   * @return true if all pose information is zero
   **/
  bool is_zero () const;

  /**
   * Tests for exact equality
   *
   * @param rhs the other pose to test against
   * @return true if all values equal corresponding values in other pose
   **/
  bool operator== (const BasicPose &rhs) const;

  /**
   * Get the name of this coordinate type
   *
   * @return "Pose"
   **/
  static std::string name ();

  /**
   * Get the number of values this coordinate type uses
   *
   * @return 6
   **/
  int size () const;

  /**
   * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
   *
   * @param i the index
   * @return the i'th coordinate value
   * @throws std::range_error if index is less than 0, or greater than 6
   **/
  double get (int i) const;

  /**
   * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
   *
   * @param i the index
   * @param val the new value
   * @return the new i'th coordinate value
   * @throws std::range_error if index is less than 0, or greater than 6
   **/
  double set (int i, double val);

  typedef BasicPose BaseType;

  /**
   * Gets a reference to this object. Useful for derived types.
   *
   * @return reference to this object.
   **/
  BaseType &as_vec ();

  /**
   * Gets a const reference to this object. Useful for derived types.
   *
   * @return const reference to this object.
   **/
  const BaseType &as_vec () const;

  /**
   * Gets a reference to this object's Position part.
   *
   * @return reference to the PositionVector
   **/
  PositionVector &as_position_vec ();
  PositionVector &as_location_vec ();

  /**
   * Gets a const reference to this object's Position part.
   *
   * @return const reference to the PositionVector
   **/
  const PositionVector &as_position_vec () const;
  const PositionVector &as_location_vec () const;

  /**
   * Gets a reference to this object's Orientation part.
   *
   * @return reference to the OrientationVector
   **/
  OrientationVector &as_orientation_vec ();

  /**
   * Gets a const reference to this object's Orientation part.
   *
   * @return const reference to the OrientationVector
   **/
  const OrientationVector &as_orientation_vec () const;

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
  template<typename Derived2>
  bool approximately_equal(const BasicPose<Derived2> &other,
      double epsilon) const
  {
    (void)other;(void)epsilon;
    throw "unimplemented";
    //return std::fabs(self().distance_to(other.self())) < epsilon;
    return false;
  }

  template<typename Derived2>
  double distance_to(const BasicPose<Derived2> &target) const {
    (void)target;
    throw "unimplemented";
    //return (target.vec() - vec()).norm();
    return 0;
  }

  /**
  * Returns a string of the values x, y, z, rx, ry, rz
  * @param delimiter         delimiter between values
  * @param unset_identifier  if true, include unset values
  * @return  stringified version of the Pose
  **/
  std::string to_string (
    const std::string & delimiter = ", ",
    const std::string & unset_identifier = "<unset>") const;

  /**
  * Saves the pose to a MADARA container
  * @param  container the container to save to
  **/
  void to_container (
    madara::knowledge::containers::NativeDoubleVector &container) const;

  /**
  * Imports the pose from a MADARA container
  * @param  container the container to import from
  **/
  void from_container (
    const madara::knowledge::containers::NativeDoubleVector &container);

  /**
  * Imports the pose from a STL vector container
  * @param  container the container to import from
  **/
  void from_container (
    const std::vector <double> &container);
};

class PoseVector :
  public BasicPose<PoseVector>
{
  using Base = BasicPose<PoseVector>;
  using Base::Base;

  static constexpr const char *type_name = "PoseVector";
};

class Pose :
  public Framed<BasicPose<Pose>>
{
public:
  using Base = Framed<BasicPose<Pose>>;
  using Base::Base;

  static constexpr const char *type_name = "Pose";

  Pose() = default;

  Pose(const Position &pos)
    : Base(pos.frame(), PositionVector(pos)) {}

  Pose(const StampedPosition &pos)
    : Base(pos.frame(), PositionVector(pos)) {}

  Pose(const Orientation &ori)
    : Base(ori.frame(), OrientationVector(ori)) {}

  Pose(const StampedOrientation &ori)
    : Base(ori.frame(), OrientationVector(ori)) {}

  Pose(const Position &pos, const Orientation &ori)
    : Base(pos.frame(), PositionVector(pos), OrientationVector(ori)) {}

  Pose(const StampedPosition &pos, const StampedOrientation &ori)
    : Base(pos.frame(), PositionVector(pos), OrientationVector(ori)) {}

  /**
   * Finds angle to the target; transforms target to this frame if needed.
   *
   * @param target the pose with the target orientation
   * @return shortest angle to map this pose's orientation onto the other pose
   **/
  double angle_to (const Pose &target) const;

  /**
   * Finds angle to the target; transforms target to this frame if needed.
   *
   * @param target the target orientation
   * @return shortest angle to map this pose's onto the given orientation
   **/
  double angle_to (const Orientation &target) const;

  /**
   * Finds angle to the target; transforms target to this frame if needed.
   *
   * @param target the pose with the target orientation
   * @return shortest angle to map this pose's orientation onto the other pose
   **/
  template<typename U> double angle_to (const Pose &target, U u) const;

  /**
   * Finds angle to the target; transforms target to this frame if needed.
   *
   * @param target the target orientation
   * @return shortest angle to map this pose's onto the given orientation
   **/
  template<typename U> double angle_to (const Orientation &target, U u) const;

  /**
   * Casting operator to extract Position from this Pose
   *
   * @return Position with same frame as this pose, and same x/y/z values
   **/
  operator Position () const;

  /**
   * Castig operator to extract Orientation from this Pose
   *
   * @return Orientation with same frame as this pose, and same rx/ry/rz values
   **/
  operator Orientation () const;
};

class StampedPose :
  public Stamped<Framed<BasicPose<StampedPose>>> 
{
  using Base = Stamped<Framed<BasicPose<StampedPose>>>;
  using Base::Base;

  static constexpr const char *type_name = "StampedPose";

  StampedPose() = default;

  StampedPose(const Position &pos)
    : Base(pos.frame(), PositionVector(pos)) {}

  StampedPose(const StampedPosition &pos)
    : Base(pos.time(), pos.frame(), PositionVector(pos)) {}

  StampedPose(const Orientation &ori)
    : Base(ori.frame(), OrientationVector(ori)) {}

  StampedPose(const StampedOrientation &ori)
    : Base(ori.time(), ori.frame(), OrientationVector(ori)) {}

  StampedPose(const Position &pos, const Orientation &ori)
    : Base(pos.frame(), PositionVector(pos), OrientationVector(ori)) {}

  StampedPose(const StampedPosition &pos, const StampedOrientation &ori)
    : Base(pos.time(), pos.frame(),
        PositionVector(pos), OrientationVector(ori)) {}

  /**
   * Casting operator to extract StampedPosition from this Pose
   *
   * @return Position with same frame as this pose, and same x/y/z values
   * @return StampedPosition with same frame and timestamp as this pose,
   *         and same x/y/z values
   **/
  operator StampedPosition () const
  {
    return StampedPosition(time(), frame(), this->as_position_vec());
  }

  /**
   * Castig operator to extract StampedOrientation from this Pose
   *
   * @return StampedOrientation with same frame and timestamp as this pose,
   *         and same rx/ry/rz values
   **/
  operator StampedOrientation () const
  {
    return StampedOrientation(time(), frame(), this->as_orientation_vec());
  }
};

template<typename Derived>
inline std::ostream &operator<<(std::ostream &o, const BasicPose<Derived> &pose)
{
  o << Derived::type_name << "(";
  o << pose.pos_vec();
  o << ";";
  o << pose.ori_vec();
  o << ")";
  return o;
}

} }

#endif

#ifndef GAMS_NO_INL
#include "Pose.inl"
#endif
