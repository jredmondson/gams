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

#ifndef _GAMS_POSE_POSE_H_
#define _GAMS_POSE_POSE_H_

#include <iostream>
#include <string>

#include <gams/pose/Position.h>
#include <gams/pose/Orientation.h>

namespace gams
{
  namespace pose
  {
    /**
     * Container for Pose information, not bound to a frame.
     * See PositionVector and OrientationVector for representation deatils
     **/
    class PoseVector : public PositionVector, public OrientationVector
    {
    public:
      /**
       * Constructs a PoseVector from individual position and orientation values
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of orientation vector along x-axis
       * @param ry length of orientation vector along y-axis
       * @param rz length of orientation vector along z-axis
       **/
      constexpr PoseVector (double x, double y, double z,
                            double rx, double ry, double rz);

      /**
       * Construct a PoseVector from a PositionVector. Orientation info will
       * be all zeros (is_orientation_zero () == true)
       *
       * @param loc the PositionVector to get position info from.
       **/
      constexpr PoseVector (const PositionVector &loc);

      /**
       * Construct a PoseVector from a OrientationVector. Position info will
       * be all zeros (is_position () == true)
       *
       * @param rot the OrientationVector to get position info from.
       **/
      constexpr PoseVector (const OrientationVector &rot);

      /**
       * Construct from individual PositionVector and OrientationVector
       *
       * @param loc the PositionVector
       * @param rot the OrientationVector
       **/
      constexpr PoseVector (const PositionVector &loc,
                            const OrientationVector &rot);

      /**
       * Default constructor. All values will be INVAL_COORD
       **/
      constexpr PoseVector ();

      /**
       * Constructor from MADARA DoubleVector
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      explicit PoseVector (
        const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      explicit PoseVector (
        const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from two MADARA DoubleVectors, for position and orientation
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      explicit PoseVector (
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot);

      /**
       * Constructor from two MADARA NativeDoubleVector, for position/orientation
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      explicit PoseVector (
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot);

      /**
       * Tests if this Pose is invalid; i.e., any values are INVAL_COORD
       *
       * @return true if at least one value is INVAL_COORD
       **/
      constexpr bool is_set () const;

      /**
      * Tests if the position is set (valid).
      *
      * @return true if position has been set to something valid
      **/
      constexpr bool is_position_set () const;
      constexpr bool is_location_set () const;

      /**
      * Tests if the orientation/orientation has been set
      *
      * @return true if orientation has been set to something valid
      **/
      constexpr bool is_orientation_set () const;

      /**
       * Tests if all position information is zero.
       *
       * @return true if all position information is zero
       **/
      constexpr bool is_position_zero () const;
      constexpr bool is_location_zero () const;

      /**
       * Tests if all orientation information is zero.
       *
       * @return true if all orientation information is zero
       **/
      constexpr bool is_orientation_zero () const;

      /**
       * Tests if all pose information is zero.
       * If true, is_position_zero and is_orientation_zero are also true
       *
       * @return true if all pose information is zero
       **/
      constexpr bool is_zero () const;

      /**
       * Tests for exact equality
       *
       * @param rhs the other pose to test against
       * @return true if all values equal corresponding values in other pose
       **/
      constexpr bool operator== (const PoseVector &rhs) const;

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
      constexpr int size () const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       *
       * @param i the index
       * @return the i'th coordinate value
       * @throws std::range_error if index is less than 0, or greater than 6
       **/
      constexpr double get (int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z, rx, ry, rz
       *
       * @param i the index
       * @param val the new value
       * @return the new i'th coordinate value
       * @throws std::range_error if index is less than 0, or greater than 6
       **/
      double set (int i, double val);

      typedef PoseVector BaseType;

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
      constexpr const BaseType &as_vec () const;

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
      constexpr const PositionVector &as_position_vec () const;
      constexpr const PositionVector &as_location_vec () const;

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
      constexpr const OrientationVector &as_orientation_vec () const;
    };

    /**
     * Represents a combination of Position and Orientation within a single
     * reference frame.
     **/
    class GAMSExport Pose : public PoseVector, public Coordinate<Pose>
    {
    public:
      /**
       * Constructs a Pose from individual position and orientation values
       * in the default frame
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of orientation vector along x-axis
       * @param ry length of orientation vector along y-axis
       * @param rz length of orientation vector along z-axis
       **/
      Pose (double x, double y, double z, double rx, double ry, double rz);

      /**
       * Constructs a Pose from individual position values in the default
       * frame. All orientation values are zero.
       *
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       **/
      Pose (double x, double y, double z = 0.0);

      /**
       * Constructs a Pose from individual position and orientation values
       * in the given frame
       *
       * @param frame the frame this pose belongs to
       * @param x position along x-axis
       * @param y position along y-axis
       * @param z position along z-axis
       * @param rx length of orientation vector along x-axis
       * @param ry length of orientation vector along y-axis
       * @param rz length of orientation vector along z-axis
       **/
      constexpr Pose (const ReferenceFrame &frame,
                     double x, double y, double z,
                     double rx, double ry, double rz);

      /**
       * Constructs a Pose from individual position values in the default
       * frame. All orientation values are zero.
       *
       * @param frame the frame this pose belongs to
       * @param x position along x-axis
       * @param frame the frame this pose belongs to
       * @param y position along y-axis
       * @param z position along z-axis
       **/
      constexpr Pose (const ReferenceFrame &frame,
                     double x, double y, double z = 0.0);

      /**
       * Default constructor. Invalid pose, in default frame
       **/
      Pose ();

      /**
       * Construct from a Position. All orientation info set to zero.
       * Frame is same as the input position.
       *
       * @param loc the Position to copy position info from.
       **/
      constexpr Pose (const Position &loc);

      /**
       * Construct from a Orientation. All position info set to zero.
       * Frame is same as the input orientation.
       *
       * @param rot the Orientation to copy orientation info from.
       **/
      constexpr Pose (const Orientation &rot);

      /**
       * Construct from individual Position and Orientation vectors, in the
       * default frame.
       *
       * @param loc the Position to copy position info from.
       * @param rot the Orientation to copy orientation info from.
       **/
      Pose (const PositionVector &loc, const OrientationVector &rot);

      /**
       * Construct from individual Position and Orientation vectors, in a
       * given frame.
       *
       * @param frame the frame this pose belongs to
       * @param loc the Position to copy position info from.
       * @param rot the Orientation to copy orientation info from.
       **/
      constexpr Pose (const ReferenceFrame &frame,
                     const PositionVector &loc,
                     const OrientationVector &rot);

      /**
       * Construct from individual Position and Orientation.
       * Frame is same as the input Position. No transformation is done.
       *
       * The Orientation's frame is ignored; its values are taken directly
       *
       * @param loc the Position to copy position info from.
       * @param rot the Orientation to copy orientation info from.
       **/
      constexpr Pose (const Position &loc, const Orientation &rot);

      /**
       * Copy constructor, but also transform to the new frame.
       *
       * @param new_frame the frame to transform into
       * @param orig      the origin of the pose
       **/
      Pose (const ReferenceFrame &new_frame, const Pose &orig);

      /**
       * Constructor from MADARA DoubleVector, into default ReferenceFrame
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      explicit Pose (
              const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA DoubleVector, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      Pose (const ReferenceFrame &frame,
               const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into default
       * ReferenceFrame
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      explicit Pose (
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into specified
       * ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (index 0, 1, 2, 3, 4, 5 go
       *            to x, y, z, rx, ry, rz)
       **/
      Pose (const ReferenceFrame &frame,
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from two MADARA DoubleVector, for position/orientation
       * into default reference frame
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      Pose (
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot);

      /**
       * Constructor from two MADARA DoubleVector, for position/orientation
       * into specified reference frame
       * @param frame the frame to belong to
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      Pose (const ReferenceFrame &frame,
        const madara::knowledge::containers::DoubleVector &vec_loc,
        const madara::knowledge::containers::DoubleVector &vec_rot);

      /**
       * Constructor from two MADARA NativeDoubleVector, for position/orientation
       * into default reference frame
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      Pose (
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot);

      /**
       * Constructor from two MADARA NativeDoubleVector, for position/orientation
       * into specified reference frame
       * @param frame the frame to belong to
       * @param vec_loc  position values from (0, 1, 2 into x, y, z)
       * @param vec_rot  orientation values from (0, 1, 2 into rx, ry, rz)
       **/
      Pose (const ReferenceFrame &frame,
        const madara::knowledge::containers::NativeDoubleVector &vec_loc,
        const madara::knowledge::containers::NativeDoubleVector &vec_rot);

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
       * Casting operator to extract Position from this Pose
       *
       * @return Position with same frame as this pose, and same x/y/z values
       **/
      constexpr operator Position () const;

      /**
       * Castig operator to extract Orientation from this Pose
       *
       * @return Orientation with same frame as this pose, and same rx/ry/rz values
       **/
      constexpr operator Orientation () const;

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
  }
}

// Include if not already included
//#include <gams/pose/ReferenceFrame.h>

#endif

#ifndef GAMS_NO_INL
#include "Pose.inl"
#endif
