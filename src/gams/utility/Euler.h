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
 * @file Euler.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Euler class, useful for specifying angles as euler
 * angles (a sequence of rotations about specified axes). To manipulate
 * these angles, convert them to the Quaternion or Rotation classes.
 **/

#ifndef _GAMS_UTILITY_EULER_H_
#define _GAMS_UTILITY_EULER_H_

#include <iostream>
#include <cmath>
#include "Rotation.h"
#include "Quaternion.h"

namespace gams
{
  namespace utility
  {
    namespace euler
    {
      /**
       * This namespace holds type tags used by the Euler class template to
       * specify a Euler convention. The euler namespace provides typedefs
       * for all possible conventions which may be simpler than using these
       * tags directly.
       **/
      namespace conv
      {
        struct Intr
        {
          static const char *name() { return "Intr"; }

          static const bool reverse = false;
        };

        struct Extr
        {
          static const char *name() { return "Extr"; }

          /* Conversion formulas are in terms of intrinsic convention; to use
           * them for extrinsic convention, we just need to pass the arguments
           * in reverse order. This flag communicates this need. */
          static const bool reverse = true;
        };

        struct X
        {
          static char *name() { return "X"; }
        };

        struct Y
        {
          static char *name() { return "Y"; }
        };

        struct Z
        {
          static char *name() { return "Z"; }
        };
      } // namespace conv
    }
  }
}

#include "gams/utility/EulerFormulas.inl"

namespace gams
{
  namespace utility
  {
    namespace euler
    {
      /// Radians unit flag; see Euler constructor
      static const detail::radians_t radians;

      /// Degres unit flag; see Euler constructor
      static const detail::degrees_t degrees;

      /// Revolutions (i.e., 1 == 360 degrees) unit flag; see Euler constructor
      static const detail::revolutions_t revolutions;

      /**
       * Class template for representing an angle in Euler notation. Euler
       * notation represents an angle as a sequence of rotations about axes
       * applied in an order. The order of rotations, and whether they are
       * applied to axes that follow the object as it rotates, or stay fixed
       * throughout, are the Euler convention. The template parameters of
       * this class template specify the convention, using type tags from the
       * "conv" namespace.
       *
       * There are typedefs for all supported conventions provided for
       * convenience. In particular, EulerVREP, and YawPitchRoll may be of
       * most common utility.
       *
       * Each convention requires specific support in the code. See
       * EulerFormulas.inl for implementation details. If you use an unsupported
       * convention, you will get a compile time error.
       *
       * Avoid manipulating angles in this notation. It is better to convert to
       * a Quaternion (or Rotation) and back, than to try to directly manipulate
       * a Euler angle.
       *
       * @tparam A the first axis of rotation
       * @tparam B the second axis of rotation
       * @tparam C the third axis of rotation
       * @tparam Conv whether to use Extrinsic (conv::Extr) or Intrisic
       *              (conv::Intr) rotations in this convention. Default is Intr
       **/
      template<typename A, typename B, typename C, typename Conv = conv::Intr>
      class Euler
      {
      private:
        typedef typename detail::GetTypes<A,B,C,Conv>::F F;
        typedef typename detail::GetTypes<A,B,C,Conv>::Trig Trig;
      public:
        /**
         * Default constructor. Initializes angles to all zeroes (no rotation).
         **/
        Euler() : a_(0), b_(0), c_(0) {}

        /**
         * Constructor from radians.
         *
         * @param a the rotation in radians around axis A
         * @param b the rotation in radians around axis B
         * @param c the rotation in radians around axis C
         **/
        Euler(double a, double b, double c) : a_(a), b_(b), c_(c) {}

        /**
         * Constructor from specified units.
         *
         * @param u the units flag (degrees, radians, or revolutions)
         * @param a the rotation around axis A
         * @param b the rotation around axis B
         * @param c the rotation around axis C
         *
         * @tparam Unit the type of the units flag (inferred automatically)
         **/
        template<typename Unit>
        Euler(Unit u, double a, double b, double c)
          : a_(u.to_radians(a)), b_(u.to_radians(b)), c_(u.to_radians(c)) {}

        /**
         * Constructor to convert from a unit Quaternion.
         * The unit-ness of the Quaternion is not verified. If a non-unit
         * Quaternion is passed in, behaviour is undefined.
         *
         * @param quat the Quaternion to convert. Must be a unit quaternion,
         *             or behavior is undefined.
         **/
        explicit Euler(const Quaternion &quat)
          : a_(Conv::reverse ? F::eular_c(quat) : F::eular_a(quat)),
            b_(F::eular_b(quat)),
            c_(Conv::reverse ? F::eular_a(quat) : F::eular_c(quat)) {}

        /**
         * Constructor to convert from a different Euler convention.
         *
         * @param o the other Euler object
         **/
        template<typename A2, typename B2, typename C2, typename Conv2>
        explicit Euler(const Euler<A2, B2, C2, Conv2> &o)
        {
          Quaternion quat(o.to_quat());
          a_ = Conv::reverse ? F::eular_c(quat) : F::eular_a(quat);
          b_ = F::eular_b(quat);
          c_ = Conv::reverse ? F::eular_a(quat) : F::eular_c(quat);
        }

        /**
         * Constructor to convert from a Rotation (or RotationVector)
         *
         * @param o the rotation
         **/
        explicit Euler(const RotationVector &r)
        {
          Quaternion quat(r);
          a_ = Conv::reverse ? F::eular_c(quat) : F::eular_a(quat);
          b_ = F::eular_b(quat);
          c_ = Conv::reverse ? F::eular_a(quat) : F::eular_c(quat);
        }

        /// Getter for the first rotation angle, around axis A
        double a() const { return a_; }
        /// Getter for the first rotation angle, around axis B
        double b() const { return b_; }
        /// Getter for the first rotation angle, around axis C
        double c() const { return c_; }

        /// Setter for the first rotation angle, around axis A
        void a(double n) { a_ = n; }
        /// Setter for the first rotation angle, around axis B
        void b(double n) { b_ = n; }
        /// Setter for the first rotation angle, around axis C
        void c(double n) { c_ = n; }

        /**
         * Convert this Euler angle to a Quaternion
         *
         * @return the Quaternion which represents the same angle as *this
         **/
        Quaternion to_quat() const
        {
          // Precalculates and caches all sin/cos required by F::quat_*
          Trig trig(Conv::reverse ? c_ : a_, b_, Conv::reverse ? a_ : c_);
          return Quaternion(F::quat_x(trig), F::quat_y(trig),
                            F::quat_z(trig), F::quat_w(trig));
        };

        /**
         * Convert this Euler angle to a Rotation (axis-angle notation),
         * within the default frame.
         *
         * @return the Rotation which represents the same angle as *this
         **/
        Rotation to_rotation() const
        {
          return Rotation(to_quat());
        };

        /**
         * Convert this Euler angle to a Rotation (axis-angle notation),
         * within the specified frame.
         *
         * @param frame the reference frame the Rotation will belong to
         * @return the Rotation which represents the same angle as *this
         **/
        Rotation to_rotation(const ReferenceFrame &frame) const
        {
          return Rotation(frame, to_quat());
        };

        /**
         * Convert a Quaternion into a Euler angle.
         *
         * @param quat the Quaternion to convert
         * @return a Euler angle equivalent to quat
         **/
        static Euler from_quat(const Quaternion &quat)
        {
          return Euler(quat);
        };
      private:
        double a_, b_, c_;
      };

      typedef Euler<conv::X, conv::Y, conv::Z>             EulerXYZ;
      typedef Euler<conv::X, conv::Y, conv::Z, conv::Intr> EulerIntrXYZ;
      typedef Euler<conv::X, conv::Y, conv::Z, conv::Extr> EulerExtrXYZ;
      typedef Euler<conv::Z, conv::Y, conv::X>             EulerZYX;
      typedef Euler<conv::Z, conv::Y, conv::X, conv::Intr> EulerIntrZYX;
      typedef Euler<conv::Z, conv::Y, conv::X, conv::Extr> EulerExtrZYX;

      /** The Euler convention used by VREP */
      typedef EulerIntrXYZ EulerVREP;

      /** The most commonly used Euler convention: Yaw-Pitch-Roll */
      typedef EulerIntrZYX EulerYPR;

      /** The most commonly used Euler convention: Yaw-Pitch-Roll */
      typedef EulerYPR YawPitchRoll;

      /** Stream operator for Euler angles */
      template<typename A, typename B, typename C, typename Conv>
      std::ostream &operator<<(std::ostream &o, const Euler<A, B, C, Conv> &e)
      {
        return o << "Euler" <<
                 Conv::name() << A::name() << B::name() << C::name() << "(" <<
                 e.a() << ", " << e.b() << ", " << e.c() << ")";
      }
    }
  }
}

#endif
