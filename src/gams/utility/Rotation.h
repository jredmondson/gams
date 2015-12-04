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
#include <string>
#include <gams/utility/Coordinate.h>
#include <cmath>

#define DEG_TO_RAD(x) (((x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) (((x) * 180) / M_PI)

namespace gams
{
  namespace utility
  {
    class ReferenceFrame;

    class Quaternion;

    /**
     * Container for Rotation information, not bound to a frame. Uses axis-angle
     * notation: a rotation is represented by a vector whose direction forms the
     * axis of rotation, with angle of rotation equal to length of the vector.
     *
     * All rotations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the rotation axis, rotations curve in the direction your
     * fingers are pointing.
     **/
    class RotationVector
    {
    public:
      static const int X_axis = 0;
      static const int Y_axis = 1;
      static const int Z_axis = 2;

      /**
       * Constructor, taking values forming an angle-axis 3-vector
       * Rotation axis is determined by vector's direction. Amount of rotation,
       * in radians, is the length of the vector.
       *
       * @param rx length of rotation vector in x-axis direction
       * @param ry length of rotation vector in y-axis direction
       * @param rz length of rotation vector in z-axis direction
       **/
      constexpr RotationVector(double rx, double ry, double rz);

      /**
       * Constructor, taking axis and angle separately
       * @pre (x, y, z) must form a unit vector
       *
       * @param x length of rotation axis unit vector in x-axis direction
       * @param y length of rotation axis unit vector in y-axis direction
       * @param z length of rotation axis unit vector in z-axis direction
       * @param angle the amount of rotation, in degrees, around the axis
       **/
      constexpr RotationVector(double x, double y, double z, double angle);

      /**
       * Constructor from MADARA DoubleVector
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit RotationVector(
        const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit RotationVector(
        const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       *
       * @param axis_index the index of the axis, 0, 1, or 2, for x, y, and z
       *    axes respectively, or constant X_axis, Y_axis, or Z_axis
       * @param angle the rotation, in degrees, around the selected axis
       **/
      constexpr RotationVector(int axis_index, double angle);

      /**
       * Default Constructor. Produces an invalid rotation (INVAL_COORD)
       **/
      constexpr RotationVector();

      /**
       * Constructor to convert from Quaternion to equivalent
       * rotation vector
       **/
      explicit RotationVector(const Quaternion &quat);

      /**
       * Tests if this object is invalid.
       *
       * @return true if any value is INVAL_COORD
       **/
      constexpr bool is_invalid() const;

      /**
       * Tests if all of rx, ry, and rz are zero
       *
       * @return true of all values are zero
       **/
      constexpr bool is_zero() const;

      /**
       * Tests for perfect equality with another RotationVector
       *
       * @param rhs the other vector to compare to
       * @return true if all values are equal to corresponding values
       **/
      constexpr bool operator==(const RotationVector &rhs) const;

      static std::string name();

      /**
       * Getter for rx
       *
       * @return rx value
       **/
      constexpr double rx() const;

      /**
       * Getter for ry
       *
       * @return ry value
       **/
      constexpr double ry() const;

      /**
       * Getter for rz
       *
       * @return rz value
       **/
      constexpr double rz() const;

      /**
       * Setter for rx
       *
       * @param new_rx the new rx value
       * @return new rx value
       **/
      double rx(double new_rx);

      /**
       * Setter for ry
       *
       * @param new_ry the new ry value
       * @return new ry value
       **/
      double ry(double new_ry);

      /**
       * Setter for rz
       *
       * @param new_rz the new rz value
       * @return new rz value
       **/
      double rz(double new_rz);

      typedef RotationVector BaseType;

      /**
       * Retrieves a reference to this type. Useful for derived types.
       *
       * @return reference to this object
       **/
      BaseType &as_vec();

      /**
       * Retrieves a const reference to this type. Useful for derived types.
       *
       * @return const reference to this object
       **/
      constexpr const BaseType &as_vec() const;

      /**
       * Gets the size of the vector this coordinate type is represented by.
       *
       * @return 3
       **/
      constexpr int size() const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order rx, ry, rz
       *
       * @param i the index
       * @return the i'th value
       **/
      constexpr double get(int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order rx, ry, rz
       *
       * @param i the index
       * @param val the new value
       * @return the new i'th value
       **/
      double set(int i, double val);

      friend class Quaternion;

      friend class ReferenceFrame;
    private:
      double rx_, ry_, rz_;
    };

    /**
     * Represents a rotation or orientation within a reference frame.
     * Uses axis-angle notation: a rotation is represented by a vector whose
     * direction forms the axis of rotation, with angle of rotation equal to
     * length of the vector.
     *
     * All rotations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the rotation axis, rotations curve in the direction your
     * fingers are pointing.
     **/
    class Rotation : public RotationVector, public Coordinate<Rotation>
    {
    public:
      /**
       * Constructor, for default frame, taking values forming an angle-axis
       * 3-vector Rotation axis is determined by vector's direction. Amount of
       * rotation, in radians, is the length of the vector.
       *
       * @param rx length of rotation vector in default-frame's x-axis direction
       * @param ry length of rotation vector in default-frame's y-axis direction
       * @param rz length of rotation vector in default-frame's z-axis direction
       **/
      Rotation(double rx, double ry, double rz);

      /**
       * Constructor, for given frame, taking values forming an angle-axis
       * 3-vector Rotation axis is determined by vector's direction. Amount of
       * rotation, in radians, is the length of the vector.
       *
       * @param frame the frame that this rotation belongs to
       * @param rx length of rotation vector in owning-frame's x-axis direction
       * @param ry length of rotation vector in owning-frame's y-axis direction
       * @param rz length of rotation vector in owning-frame's z-axis direction
       **/
      constexpr Rotation(const ReferenceFrame &frame,
                         double rx, double ry, double rz);

      /**
       * Constructor, for default frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt(x*x + y*y + z*z) == 1)
       *
       * @param x length of rotation axis vector in frame's x-axis direction
       * @param y length of rotation axis vector in frame's y-axis direction
       * @param z length of rotation axis vector in frame's z-axis direction
       * @param angle the amount of rotation, in degrees, around the axis
       **/
      Rotation(double x, double y, double z, double angle);

      /**
       * Constructor, for a given frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt(x*x + y*y + z*z) == 1)
       *
       * @param frame the frame that this rotation belongs to
       * @param x length of rotation axis vector in frame's x-axis direction
       * @param y length of rotation axis vector in frame's y-axis direction
       * @param z length of rotation axis vector in frame's z-axis direction
       * @param angle the amount of rotation, in degrees, around the axis
       **/
      constexpr Rotation(const ReferenceFrame &frame,
               double x, double y, double z, double angle);

      /**
       * Constructor from MADARA DoubleVector, into default ReferenceFrame
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit Rotation(
              const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA DoubleVector, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      Rotation(const ReferenceFrame &frame,
               const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into default
       * ReferenceFrame
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit Rotation(
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into specified
       * ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      Rotation(const ReferenceFrame &frame,
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       *
       * This Rotation will belong to the default frame.
       *
       * @param axis_index the index of the axis, 0, 1, or 2, for x, y, and z
       *    axes respectively, or constant X_axis, Y_axis, or Z_axis
       * @param angle the rotation, in degrees, around the selected axis
       **/
      Rotation(int axis_index, double angle);

      /**
       * For easy specification of a simple rotation around a single axis,
       * pass the index of the axis (0, 1, 2 for X, Y, and Z, respectively)
       * or use the X_axis, Y_axis, or Z_axis constants. Pass the rotation
       * around that axis as the angle, in degrees
       *
       * @param frame the frame that this rotation belongs to
       * @param axis_index the index of the axis, 0, 1, or 2, for x, y, and z
       *    axes respectively, or constant X_axis, Y_axis, or Z_axis
       * @param angle the rotation, in degrees, around the selected axis
       **/
      constexpr Rotation(const ReferenceFrame &frame,
                         int axis_index, double angle);

      /**
       * Default constructor; an invalid rotation, in the default frame
       **/
      Rotation();

      /**
       * Construct from a Quaternion, into the default frame.
       *
       * @param quat the Quaternion to converto to a rotation vector
       **/
      explicit Rotation(const Quaternion &quat);

      /**
       * Construct from a Quaternion, into a given frame.
       *
       * @param frame the frame that this rotation belongs to
       * @param quat the Quaternion to converto to a rotation vector
       **/
      explicit Rotation(const ReferenceFrame &frame, const Quaternion &quat);

      /**
       * Copy constructor, but also convert to a new frame
       *
       * @param new_frame the new frame to convert to
       **/
      Rotation(const ReferenceFrame &new_frame, const Rotation &orig);

      /**
       * Synonym for distance_to. Returns angle of shortest rotation mapping
       * this rotation onto the target.
       *
       * @param target the other Rotation
       * @return the shortest angle to rotate this onto target
       **/
      double angle_to(const Rotation &target) const;

      using Coordinate<Rotation>::operator==;
    };
  }
}

#include "Rotation.inl"

// Include if not already included
#include <gams/utility/Pose.h>

#endif
