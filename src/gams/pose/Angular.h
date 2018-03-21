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
 * @file Angular.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Angular and AngularVector classes
 **/

namespace gams
{
  namespace pose
  {
    template<class C>
    class Angular;
  }
}

#ifndef _GAMS_POSE_ANGULAR_H
#define _GAMS_POSE_ANGULAR_H

#include <iostream>
#include <string>
#include <gams/CPP11_compat.h>
#include <gams/pose/Coordinate.h>
#include "gams/GAMSExport.h"
#include <cmath>
#include <madara/knowledge/containers/DoubleVector.h>
#include <madara/knowledge/containers/NativeDoubleVector.h>

#define DEG_TO_RAD(x) ( ( (x) * M_PI) / 180.0)
#define RAD_TO_DEG(x) ( ( (x) * 180) / M_PI)

#include "AngleUnits.h"

namespace gams
{
  namespace pose
  {
    class ReferenceFrame;

    class Quaternion;

    /**
     * Container for Angular information, not bound to a frame. Uses axis-angle
     * notation: a orientation is represented by a vector whose direction forms the
     * axis of orientation, with angle of orientation equal to length of the vector.
     *
     * All orientations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the orientation axis, orientations curve in the direction your
     * fingers are pointing.
     **/
    class AngularVector
    {
    public:
      /**
       * Constructor, taking values forming an angle-axis 3-vector
       * Angular axis is determined by vector's direction. The length of
       * the vector is the amount of orientation (in radians, by default).
       *
       * @param rx length of orientation vector in x-axis direction
       * @param ry length of orientation vector in y-axis direction
       * @param rz length of orientation vector in z-axis direction
       **/
      constexpr AngularVector (double rx, double ry, double rz);

      /**
       * Constructor, taking axis and angle separately
       * @pre (x, y, z) must form a unit vector
       *
       * @param x length of orientation axis unit vector in x-axis direction
       * @param y length of orientation axis unit vector in y-axis direction
       * @param z length of orientation axis unit vector in z-axis direction
       * @param angle the amount of orientation around the axis
       **/
      constexpr AngularVector (double x, double y, double z, double angle);

      /**
       * Constructor from MADARA DoubleVector
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit AngularVector (
        const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit AngularVector (
        const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Default Constructor. Produces an invalid orientation (INVAL_COORD)
       **/
      constexpr AngularVector ();

      /**
       * Constructor to convert from Quaternion to equivalent
       * orientation vector
       *
       * Implementation in Quaternion.inl due to circular dependencies
       **/
      explicit AngularVector (const Quaternion &quat);

      /**
       * Tests if this object is invalid.
       *
       * @return true if any value is INVAL_COORD
       **/
      constexpr bool is_set () const;

      /**
       * Tests if all of rx, ry, and rz are zero
       *
       * @return true of all values are zero
       **/
      constexpr bool is_zero () const;

      /**
       * Tests for perfect equality with another AngularVector
       *
       * @param rhs the other vector to compare to
       * @return true if all values are equal to corresponding values
       **/
      constexpr bool operator== (const AngularVector &rhs) const;

      static std::string name ();

      /**
       * Getter for rx
       *
       * @return rx value
       **/
      constexpr double rx () const;

      /**
       * Getter for ry
       *
       * @return ry value
       **/
      constexpr double ry () const;

      /**
       * Getter for rz
       *
       * @return rz value
       **/
      constexpr double rz () const;

      /**
       * Setter for rx
       *
       * @param new_rx the new rx value
       * @return new rx value
       **/
      double rx (double new_rx);

      /**
       * Setter for ry
       *
       * @param new_ry the new ry value
       * @return new ry value
       **/
      double ry (double new_ry);

      /**
       * Setter for rz
       *
       * @param new_rz the new rz value
       * @return new rz value
       **/
      double rz (double new_rz);

      typedef AngularVector BaseType;

      /**
       * Retrieves a reference to this type. Useful for derived types.
       *
       * @return reference to this object
       **/
      BaseType &as_vec ();

      /**
       * Retrieves a const reference to this type. Useful for derived types.
       *
       * @return const reference to this object
       **/
      constexpr const BaseType &as_vec () const;

      /**
       * Gets the size of the vector this coordinate type is represented by.
       *
       * @return 3
       **/
      constexpr int size () const;

      /**
       * Retrives i'th coordinate, 0-indexed, in order rx, ry, rz
       *
       * @param i the index
       * @return the i'th value
       **/
      constexpr double get (int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order rx, ry, rz
       *
       * @param i the index
       * @param val the new value
       * @return the new i'th value
       **/
      double set (int i, double val);

      friend class Quaternion;

      friend class ReferenceFrame;
    private:
      double rx_, ry_, rz_;
    };

    /**
     * Represents a orientation or orientation within a reference frame.
     * Uses axis-angle notation: a orientation is represented by a vector whose
     * direction forms the axis of orientation, with angle of orientation equal to
     * length of the vector.
     *
     * All orientations about an axis follow the right hand role; if the origin is
     * the center of your right hand, and your thumb is pointing in the positive
     * direction of the orientation axis, orientations curve in the direction your
     * fingers are pointing.
     **/
    template<class C>
    class Angular : public AngularVector, public Coordinate<C>
    {
    public:
      /**
       * Constructor, for default frame, taking values forming an angle-axis
       * 3-vector Angular axis is determined by vector's direction. Amount of
       * orientation, in radians, is the length of the vector.
       *
       * For a simple orientation around a single axis, just pass that angle in the
       * corresponding axis' argument.
       *
       * @param rx length of orientation vector in default-frame's x-axis direction
       * @param ry length of orientation vector in default-frame's y-axis direction
       * @param rz length of orientation vector in default-frame's z-axis direction
       **/
      Angular (double rx, double ry, double rz);

      /**
       * Constructor, for default frame, taking values forming an angle-axis
       * 3-vector Angular axis is determined by vector's direction. Amount of
       * orientation is the length of the vector.
       *
       * For a simple orientation around a single axis, just pass that angle in the
       * corresponding axis' argument.
       *
       * @param rx length of orientation vector in default-frame's x-axis direction
       * @param ry length of orientation vector in default-frame's y-axis direction
       * @param rz length of orientation vector in default-frame's z-axis direction
       * @param u units to use (see AngleUnits.h)
       **/
      template<typename U> Angular (double rx, double ry, double rz, U u);

      /**
       * Constructor, for given frame, taking values forming an angle-axis
       * 3-vector Angular axis is determined by vector's direction. Amount of
       * orientation, in radians, is the length of the vector.
       *
       * For a simple orientation around a single axis, just pass that angle in the
       * corresponding axis' argument.
       *
       * @param frame the frame that this orientation belongs to
       * @param rx length of orientation vector in owning-frame's x-axis direction
       * @param ry length of orientation vector in owning-frame's y-axis direction
       * @param rz length of orientation vector in owning-frame's z-axis direction
       **/
      constexpr Angular (const ReferenceFrame &frame,
                         double rx, double ry, double rz);

      /**
       * Constructor, for given frame, taking values forming an angle-axis
       * 3-vector Angular axis is determined by vector's direction. Amount of
       * orientation is the length of the vector.
       *
       * For a simple orientation around a single axis, just pass that angle in the
       * corresponding axis' argument.
       *
       * @param frame the frame that this orientation belongs to
       * @param rx length of orientation vector in owning-frame's x-axis direction
       * @param ry length of orientation vector in owning-frame's y-axis direction
       * @param rz length of orientation vector in owning-frame's z-axis direction
       * @param u units to use (see AngleUnits.h)
       **/
      template<typename U>
      constexpr Angular (const ReferenceFrame &frame,
                         double rx, double ry, double rz, U u);

      /**
       * Constructor, for default frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt (x*x + y*y + z*z) == 1)
       *
       * @param x length of orientation axis vector in frame's x-axis direction
       * @param y length of orientation axis vector in frame's y-axis direction
       * @param z length of orientation axis vector in frame's z-axis direction
       * @param angle the amount of orientation, in radians, around the axis
       **/
      Angular (double x, double y, double z,
                                    double angle);

      /**
       * Constructor, for default frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt (x*x + y*y + z*z) == 1)
       *
       * @param x length of orientation axis vector in frame's x-axis direction
       * @param y length of orientation axis vector in frame's y-axis direction
       * @param z length of orientation axis vector in frame's z-axis direction
       * @param angle the amount of orientation around the axis
       * @param u units to use (see AngleUnits.h)
       **/
      template<typename U> Angular (double x, double y, double z,
                                    double angle, U u);

      /**
       * Constructor, for a given frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt (x*x + y*y + z*z) == 1)
       *
       * @param frame the frame that this orientation belongs to
       * @param x length of orientation axis vector in frame's x-axis direction
       * @param y length of orientation axis vector in frame's y-axis direction
       * @param z length of orientation axis vector in frame's z-axis direction
       * @param angle the amount of orientation, default radians, around the axis
       **/
      constexpr Angular (const ReferenceFrame &frame,
                         double x, double y, double z, double angle);

      /**
       * Constructor, for a given frame taking axis and angle separately
       * @pre (x, y, z) must form a unit vector (sqrt (x*x + y*y + z*z) == 1)
       *
       * @param frame the frame that this orientation belongs to
       * @param x length of orientation axis vector in frame's x-axis direction
       * @param y length of orientation axis vector in frame's y-axis direction
       * @param z length of orientation axis vector in frame's z-axis direction
       * @param angle the amount of orientation around the axis
       * @param u units to use (see AngleUnits.h)
       **/
      template<typename U> constexpr Angular (const ReferenceFrame &frame,
                                              double x, double y, double z,
                                              double angle, U u);

      /**
       * Constructor from MADARA DoubleVector, into default ReferenceFrame
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit Angular (
              const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA DoubleVector, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      Angular (const ReferenceFrame &frame,
               const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into default
       * ReferenceFrame
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      explicit Angular (
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into specified
       * ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (0, 1, 2 go to rx, ry, rz)
       **/
      Angular (const ReferenceFrame &frame,
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Default constructor; an invalid orientation, in the default frame
       **/
      Angular ();

      /**
       * Construct from a Quaternion, into the default frame.
       *
       * @param quat the Quaternion to converto to a orientation vector
       **/
      explicit Angular (const Quaternion &quat);

      /**
       * Construct from a Quaternion, into a given frame.
       *
       * @param frame the frame that this orientation belongs to
       * @param quat the Quaternion to converto to a orientation vector
       **/
      explicit Angular (const ReferenceFrame &frame, const Quaternion &quat);

      /**
       * Copy constructor, but also convert to a new frame
       *
       * @param new_frame the new frame to convert to
       * @param orig the origin of the orientation
       **/
      Angular (const ReferenceFrame &new_frame, const C &orig);

      /**
       * Synonym for distance_to. Returns angle of shortest orientation mapping
       * this orientation onto the target.
       *
       * @param target the other Angular
       * @return the shortest angle, in radians, to orient this onto target
       **/
      double angle_to (const C &target) const;

      /**
       * Synonym for distance_to. Returns angle of shortest orientation mapping
       * this orientation onto the target.
       *
       * @param target the other Angular
       * @param u units to use (see AngleUnits.h)
       * @return the shortest angle to orient this onto target
       **/
      template<typename U> double angle_to (const C &target, U u) const;

      /**
       * Interpolate a new Angular that is (t * 100)% between *this and
       * o, along the shortest path of orientation between them in 3D space.
       *
       * @param o the other Angular. If it is not in the same frame as *this,
       *          it will be transformed first.
       * @param t between 0 and 1, how "close" the interpolated orientation should
       *          be to *this
       *
       * @return the interpolated Angular.
       *
       * Implementation in Quaternion.inl due to circular dependencies
       **/
      Angular slerp (const Angular &o, double t) const;

      /**
      * Returns a string of the values rx, ry, rz
      * @param delimiter         delimiter between values
      * @param unset_identifier  value to print if unset
      * @return  stringified version of the Angular
      **/
      std::string to_string (
        const std::string & delimiter = ", ",
        const std::string & unset_identifier = "<unset>") const;

      /**
       * Interpolate a new Angular that is (t * 100)% between *this and
       * o, along the shortest path of orientation between them in 3D space.
       * Store the result in *this.
       *
       * @param o the other Angular. If it is not in the same frame as *this,
       *          it will be transformed first.
       * @param t between 0 and 1, how "close" the interpolated orientation should
       *          be to *this
       *
       * Implementation in Quaternion.inl due to circular dependencies
       **/
      void slerp_this (const Angular &o, double t);


      /**
       * Saves the orientation to a MADARA container
       * @param  container the container to save to
       **/
      void to_container (
        madara::knowledge::containers::NativeDoubleVector &container) const;

      /**
      * Imports the orientation from a MADARA container
      * @param  container the container to import from
      **/
      void from_container (
        const madara::knowledge::containers::NativeDoubleVector &container);

      using Coordinate::operator==;
    };
  }
}

#include "Angular.inl"

#endif // _GAMS_UTILITY_ANGULAR_H_
