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

#ifndef _GAMS_UTILITY_POSE_H_
#define _GAMS_UTILITY_POSE_H_

#include <iostream>
#include <string>

#include <gams/utility/Location.h>
#include <gams/utility/Rotation.h>

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    /**
     * Container for Pose information, not bound to a frame.
     * See Location_Vector and Rotation_Vector for representation deatils
     **/
    class Pose_Vector : public Location_Vector, public Rotation_Vector
    {
    public:
      /**
       * Constructs a Pose_Vector from individual location and rotation values
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of rotation vector along x-axis
       * @param ry length of rotation vector along y-axis
       * @param rz length of rotation vector along z-axis
       **/
      constexpr Pose_Vector(double x, double y, double z,
                            double rx, double ry, double rz);

      /**
       * Construct a Pose_Vector from a Location_Vector. Rotation info will
       * be all zeros (is_rotation_zero() == true)
       *
       * @param loc the Location_Vector to get location info from.
       **/
      constexpr Pose_Vector(const Location_Vector &loc);

      /**
       * Construct a Pose_Vector from a Rotation_Vector. Location info will
       * be all zeros (is_location_zero() == true)
       *
       * @param loc the Rotation_Vector to get location info from.
       **/
      constexpr Pose_Vector(const Rotation_Vector &rot);

      /**
       * Construct from individual Location_Vector and Rotation_Vector
       *
       * @param loc the Location_Vector
       * @param rot the Rotation_Vector
       **/
      constexpr Pose_Vector(const Location_Vector &loc,
                            const Rotation_Vector &rot);

      /**
       * Default constructor. All values will be INVAL_COORD
       **/
      constexpr Pose_Vector();

      /**
       * Copy constructor
       **/
      constexpr Pose_Vector(const Pose_Vector &other);

      /**
       * Tests if this Pose is invalid; i.e., any values are INVAL_COORD
       *
       * @return true if at least one value is INVAL_COORD
       **/
      constexpr bool is_invalid() const;

      /**
       * Tests if all location information is zero.
       *
       * @return true if all location information is zero
       **/
      constexpr bool is_location_zero() const;

      /**
       * Tests if all rotation information is zero.
       *
       * @return true if all rotation information is zero
       **/
      constexpr bool is_rotation_zero() const;

      /**
       * Tests if all pose information is zero.
       * If true, is_location_zero and is_rotation_zero are also true
       *
       * @return true if all pose information is zero
       **/
      constexpr bool is_zero() const;

      /**
       * Tests for exact equality
       *
       * @param rhs the other pose to test against
       * @return true if all values equal corresponding values in other pose
       **/
      constexpr bool operator==(const Pose_Vector &rhs) const;

      /**
       * Get the name of this coordinate type
       *
       * @return "Pose"
       **/
      static std::string name();

      /**
       * Get the number of values this coordinate type uses
       *
       * @return 6
       **/
      constexpr int size() const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       *
       * @param i the index
       * @return the i'th coordinate value
       * @throws std::range_error if index is less than 0, or greater than 6
       **/
      constexpr double get(int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       *
       * @param i the index
       * @param val the new value
       * @return the new i'th coordinate value
       * @throws std::range_error if index is less than 0, or greater than 6
       **/
      double set(int i, double val);

      typedef Pose_Vector Base_Type;

      /**
       * Gets a reference to this object. Useful for derived types.
       *
       * @return reference to this object.
       **/
      Base_Type &as_vec();

      /**
       * Gets a const reference to this object. Useful for derived types.
       *
       * @return const reference to this object.
       **/
      constexpr const Base_Type &as_vec() const;

      /**
       * Gets a reference to this object's Location part.
       *
       * @return reference to the Location_Vector
       **/
      Location_Vector &as_location_vec();

      /**
       * Gets a const reference to this object's Location part.
       *
       * @return const reference to the Location_Vector
       **/
      constexpr const Location_Vector &as_location_vec() const;

      /**
       * Gets a reference to this object's Rotation part.
       *
       * @return reference to the Rotation_Vector
       **/
      Rotation_Vector &as_rotation_vec();

      /**
       * Gets a const reference to this object's Rotation part.
       *
       * @return const reference to the Rotation_Vector
       **/
      constexpr const Rotation_Vector &as_rotation_vec() const;
    };

    /**
     * Represents a combination of Location and Rotation within a single
     * reference frame.
     **/
    class Pose : public Pose_Vector, public Coordinate<Pose>
    {
    public:
      /**
       * Constructs a Pose from individual location and rotation values
       * in the default frame
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of rotation vector along x-axis
       * @param ry length of rotation vector along y-axis
       * @param rz length of rotation vector along z-axis
       **/
      Pose(double x, double y, double z, double rx, double ry, double rz);

      /**
       * Constructs a Pose from individual location values in the default
       * frame. All rotation values are zero.
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       **/
      Pose(double x, double y, double z = 0.0);

      /**
       * Constructs a Pose from individual location and rotation values
       * in the given frame
       *
       * @param frame the frame this pose belongs to
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of rotation vector along x-axis
       * @param ry length of rotation vector along y-axis
       * @param rz length of rotation vector along z-axis
       **/
      constexpr Pose(const Reference_Frame &frame,
                     double x, double y, double z,
                     double rx, double ry, double rz);

      /**
       * Constructs a Pose from individual location values in the default
       * frame. All rotation values are zero.
       *
       * @param frame the frame this pose belongs to
       * @param x position along x-axis
       * @param frame the frame this pose belongs to
       * @param y position along y-axis
       * @param z position along z-axis
       **/
      constexpr Pose(const Reference_Frame &frame,
                     double x, double y, double z = 0.0);

      /**
       * Default constructor. Invalid pose, in default frame
       **/
      Pose();

      /**
       * Construct from a Location. All rotation info set to zero.
       * Frame is same as the input location.
       *
       * @param loc the Location to copy location info from.
       **/
      constexpr Pose(const Location &loc);

      /**
       * Construct from a Rotation. All location info set to zero.
       * Frame is same as the input rotation.
       *
       * @param rot the Rotation to copy rotation info from.
       **/
      constexpr Pose(const Rotation &rot);

      /**
       * Construct from individual Location and Rotation vectors, in the
       * default frame.
       *
       * @param loc the Location to copy location info from.
       * @param rot the Rotation to copy rotation info from.
       **/
      Pose(const Location_Vector &loc, const Rotation_Vector &rot);

      /**
       * Copy constructor
       **/
      constexpr Pose(const Pose &other);

      /**
       * Construct from individual Location and Rotation vectors, in a
       * given frame.
       *
       * @param frame the frame this pose belongs to
       * @param loc the Location to copy location info from.
       * @param rot the Rotation to copy rotation info from.
       **/
      constexpr Pose(const Reference_Frame &frame,
                     const Location_Vector &loc,
                     const Rotation_Vector &rot);

      /**
       * Construct from individual Location and Rotation.
       * Frame is same as the input Location. No transformation is done.
       *
       * @pre loc.frame() == rot.frame()
       *
       * @param loc the Location to copy location info from.
       * @param rot the Rotation to copy rotation info from.
       **/
      constexpr Pose(const Location &loc, const Rotation &rot);

      /**
       * Copy constructor, but also transform to the new frame.
       *
       * @param new_frame the frame to transform into
       **/
      Pose(const Reference_Frame &new_frame, const Pose &orig);

      /**
       * Finds angle to the target; transforms target to this frame if needed.
       *
       * @param target the pose with the target rotation
       * @return shortest angle to map this pose's rotation onto the other pose
       **/
      double angle_to(const Pose &target) const;

      /**
       * Finds angle to the target; transforms target to this frame if needed.
       *
       * @param target the target rotation
       * @return shortest angle to map this pose's onto the given rotation
       **/
      double angle_to(const Rotation &target) const;

      /**
       * Casting operator to extract Location from this Pose
       *
       * @return Location with same frame as this pose, and same x/y/z values
       **/
      constexpr operator Location() const;

      /**
       * Castig operator to extract Rotation from this Pose
       *
       * @return Rotation with same frame as this pose, and same rx/ry/rz values
       **/
      constexpr operator Rotation() const;
    };
  }
}

#include "Pose.inl"

// Include if not already included
#include <gams/utility/Reference_Frame.h>

#endif
