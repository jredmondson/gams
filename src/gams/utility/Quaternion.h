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
 * @file Quaternion.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Quaternion class, useful for performing angle math
 **/

#ifndef _GAMS_UTILITY_QUATERNION_H_
#define _GAMS_UTILITY_QUATERNION_H_

#include <iostream>
#include <cmath>
#include <gams/CPP11_compat.h>

namespace gams
{
  namespace utility
  {
    class OrientationVector;
    class LocationVector;

    /**
     * Used internally to implement angle operations.
     * Not reference-frame aware.
     **/
    class Quaternion
    {
    public:
      /**
       * Default constructor. Initializes x, y, z, and w to zero.
       **/
      constexpr Quaternion() : x_(0), y_(0), z_(0), w_(0) {}

      /**
       * Primary constructor. Specifies each term explicitly
       **/
      constexpr Quaternion(double x, double y, double z, double w);

      /**
       * Constructor which converts a rotation vector, specified in individual
       * terms, into the corresponding quaternion representation. See
       * OrientationVector for details of the rotation vector representation.
       **/
      Quaternion(double rx, double ry, double rz);

      /**
       * Constructor which converts a rotation vector into the corresponding
       * quaternion representation. See OrientationVector for details of the
       * rotation vector representation.
       **/
      explicit Quaternion(const OrientationVector &rot);

      /**
       * Constructor which converts a location vector into the corresponding
       * quaternion representation; x, y, and z are copied over. The w term is 0
       **/
      explicit Quaternion(const LocationVector &loc);

      void from_location_vector(double x, double y, double z);

      void from_location_vector(const LocationVector &loc);

      void to_location_vector(double &x, double &y, double &z) const;

      void to_location_vector(LocationVector &loc) const;

      void from_orientation_vector(double rx, double ry, double rz);

      void from_orientation_vector(const OrientationVector &rot);

      void to_orientation_vector(double &rx, double &ry, double &rz) const;

      void to_orientation_vector(OrientationVector &rot) const;

      /**
       * Calculates the hamilton product of two quaternions, into a third.
       * The resulting quaternion represents the composed rotation of the args.
       * The target quaternion can be one of the two source quaternion.
       * The two source quaternions can be the same (for squaring)
       *
       * @param into destination quaternion; will be overwritten
       * @param lhs left-hand-side source quaternion (non-commutative)
       * @param rhs right-hand-side source quaternion (non-commutative)
       **/
      static void hamilton_product(
              Quaternion &into, const Quaternion &lhs, const Quaternion &rhs);

      /**
       * Calculate hamilton product, this * rhs, and store into this.
       * Equivalent to hamilton_product(*this, *this, rhs)
       *
       * @param rhs right-hand-side to multiply with
       * @return *this, the result
       **/
      Quaternion &operator*=(const Quaternion &rhs);

      /**
       * Calculate hamilton product, this * rhs, and return new Quaternion
       *
       * @param rhs right-hand-side to multiply with
       * @return a new Quaternion object holding the result
       **/
      Quaternion operator*(const Quaternion &rhs) const;

      /**
       * Calculate hamilton product, lhs * this, and store into this.
       * Equivalent to hamilton_product(*this, lhs, *this)
       *
       * @param lhs left-hand-side to multiply with
       **/
      void pre_multiply(const Quaternion &lhs)
      {
        hamilton_product(*this, lhs, *this);
      }

      /**
       * Conjugate this quaternion in-place; may use in place of invert for unit
       * quaternions for better performance.
       *
       * @return *this, after conjugation
       **/
      Quaternion &conjugate();

      /**
       * Flip the sign of all parts of the quaternion; equivalent to (but
       * potentially faster than) this->scale(-1);
       *
       * @return *this, after negation
       **/
      Quaternion &negate();

      /**
       * Treat this quaternion as a location quaternion (w should be zero),
       * and rotate it by the rotation represented by rot
       *
       * @param rot the Quaternion angle to rotate by
       **/
      void rotate_by(Quaternion rot);

      /**
       * Copy and conjugate this quaternion
       *
       * @return a new Quaternion, holding the conjugation of this
       **/
      constexpr Quaternion operator-() const;

      /**
       * Calculates the Quaternion inner-product between this, and the
       * right-hand-side, which is the sum of the pair-wise
       * multiplication of each corresponding term of the Quaternions.
       * Note that unlike hamilton_product, this is commutative.
       *
       * @param rhs the Quaternion to multiply with
       * @return the scalar result.
       **/
      double inner_product(const Quaternion &rhs) const;

      /**
       * Calculates the Quaternion dot product between this, and the
       * right-hand-side; this is the inner-product, over the product
       * of the magnitude of both sides.
       *
       * @param rhs the Quaternion to dot-multiply with
       * @return the scalar result.
       **/
      double dot_product(const Quaternion &rhs) const;

      /**
       * Calculates the angle between the rotations represented by
       * this, and the target Quaternion. This is the smallest angle
       * this can be rotated to arrive at target.
       *
       * @param target the target Quaternion
       * @return the angle in radians between this and the target
       **/
      double angle_to(const Quaternion &target) const;

      /** getter of x (or i) part of quaternion */
      constexpr double x() const;

      /** getter of y (or j) part of quaternion */
      constexpr double y() const;

      /** getter of z (or k) part of quaternion */
      constexpr double z() const;

      /** getter of w (or real scalar) part of quaternion */
      constexpr double w() const;

      /** getter of x (or i) part of quaternion */
      double x(double new_x);

      /** getter of y (or j) part of quaternion */
      double y(double new_y);

      /** getter of z (or k) part of quaternion */
      double z(double new_z);

      /** setter of w (or real scalar) part of quaternion */
      double w(double new_w);

      /**
       * Inverts this quaternion to its reciprical. Equivalent to (but slower
       * than) conjugate if you know this quaternion is a unit quaternion.
       **/
      void invert();

      /**
       * Returns the magnitude of this quaternion
       **/
      double mag() const;

      /**
       * Returns the magnitude of imaginary part of this quaternion
       **/
      double imag() const;

      /**
       * Returns the magnitude of this quaternion, squared. Faster than
       * squaring the return value of mag()
       **/
      double mag_squared() const;

      /**
       * Multiply each part of this quaternion by s
       **/
      void scale(double s);

      /**
       * Calculate e to the power of this quaternion, return result as new
       * quaternion.
       *
       * @return result
       **/
      Quaternion exp() const;

      /**
       * Calculate e to the power of this quaternion, store result in this
       * quaternion.
       **/
      void exp_this();

      /**
       * Calculate the natural log of this quaternion, return result in a new
       * quaternion.
       *
       * @return result
       **/
      Quaternion ln() const;

      /**
       * Calculate natural log of this quaternion, store result in this
       * quaternion.
       **/
      void ln_this();

      /**
       * Calculate this quaternion raised to a scalar power, return result in
       * a new quaternion.
       *
       * @return result.
       **/
      Quaternion pow(double e) const;

      /**
       * Calculate this quaternion raised to a scalar power, store result in
       * this quaternion.
       **/
      void pow_this(double e);

      /**
       * Calculate this quaternion raised to a quaternion power, return result
       * in a new quaternion.
       *
       * @return result.
       **/
      Quaternion pow(const Quaternion &e) const;

      /**
       * Calculate this quaternion raised to a quaternion power, store result in
       * this quaternion.
       **/
      void pow_this(const Quaternion &e);

      /**
       * If *this and o are unit quaternions, interpolate a quaternion that is
       * partially between them, and return it as a new quaternion.
       *
       * If either quaternion is not a unit quaternion, the calculation will
       * not produce an error, but will be meaningless.
       *
       * @param o the other unit quaternion
       * @param t how "close" to this quaternion the result should be, in a
       *          range from 0 to 1, inclusive, where 0 is *this, 1 is o, and
       *          0.5 is halfway inbetween.
       * @return the resulting interpolated quaternion
       **/
      Quaternion slerp(const Quaternion &o, double t);

      /**
       * If *this and o are unit quaternions, interpolate a quaternion that is
       * partially between them, and store it in *this
       *
       * If either quaternion is not a unit quaternion, the calculation will
       * not produce an error, but will be meaningless.
       *
       * @param o the other unit quaternion
       * @param t how "close" to this quaternion the result should be, in a
       *          range from 0 to 1, inclusive, where 0 is *this, 1 is o, and
       *          0.5 is halfway inbetween.
       **/
      void slerp_this(const Quaternion &o, double t);

      /* The methods below return the cells of the 3x3 rotation matrix this
       * quaternion represents; names are mRC(), where R is row, and C is column
       */

      /** Row 1 Column 1 of rotation matrix equivalent to this quaternion */
      double m11() const;

      /** Row 1 Column 2 of rotation matrix equivalent to this quaternion */
      double m12() const;

      /** Row 1 Column 3 of rotation matrix equivalent to this quaternion */
      double m13() const;

      /** Row 2 Column 1 of rotation matrix equivalent to this quaternion */
      double m21() const;

      /** Row 2 Column 2 of rotation matrix equivalent to this quaternion */
      double m22() const;

      /** Row 2 Column 3 of rotation matrix equivalent to this quaternion */
      double m23() const;

      /** Row 3 Column 1 of rotation matrix equivalent to this quaternion */
      double m31() const;

      /** Row 3 Column 2 of rotation matrix equivalent to this quaternion */
      double m32() const;

      /** Row 3 Column 3 of rotation matrix equivalent to this quaternion */
      double m33() const;

    private:
      double x_, y_, z_, w_;
    };
  }
}

#include "Quaternion.inl"

#endif
