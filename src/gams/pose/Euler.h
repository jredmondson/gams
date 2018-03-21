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
 * angles (a sequence of angulars about specified axes). To manipulate
 * these angles, convert them to the Quaternion or Angular classes.
 **/

#ifndef _GAMS_POSE_EULER_H_
#define _GAMS_POSE_EULER_H_

#include <iostream>
#include <cmath>
#include "Angular.h"
#include "AngleUnits.h"
#include "Quaternion.h"

namespace gams
{
  namespace pose
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
          static const char *name() { return "X"; }
        };

        struct Y
        {
          static const char *name() { return "Y"; }
        };

        struct Z
        {
          static const char *name() { return "Z"; }
        };
      } // namespace conv
    }
  }
}

#include "gams/pose/EulerFormulas.inl"

namespace gams
{
  namespace pose
  {
    namespace euler
    {
      /**
       * Class template for representing an angle in Euler notation. Euler
       * notation represents an angle as a sequence of angulars about axes
       * applied in an order. The order of angulars, and whether they are
       * applied to axes that follow the object as it orients, or stay fixed
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
       * a Quaternion (or Angular) and back, than to try to directly manipulate
       * a Euler angle.
       *
       * @tparam A the first axis of angular
       * @tparam B the second axis of angular
       * @tparam C the third axis of angular
       * @tparam Conv whether to use Extrinsic (conv::Extr) or Intrisic
       *              (conv::Intr) angulars in this convention. Default is Intr
       **/
      template<typename A, typename B, typename C, typename Conv = conv::Intr>
      class Euler
      {
      private:
        typedef typename detail::GetTypes<A,B,C,Conv>::F F;
        typedef typename detail::GetTypes<A,B,C,Conv>::Trig Trig;
      public:
        /**
         * Default constructor. Initializes angles to all zeroes (no angular).
         **/
        Euler() : a_(0), b_(0), c_(0) {}

        /**
         * Constructor from radians.
         *
         * @param a the angular in radians around axis A
         * @param b the angular in radians around axis B
         * @param c the angular in radians around axis C
         **/
        Euler(double a, double b, double c) : a_(a), b_(b), c_(c) {}

        /**
         * Constructor from specified units.
         *
         * @param u the units flag (degrees, radians, or revolutions)
         * @param a the angular around axis A
         * @param b the angular around axis B
         * @param c the angular around axis C
         *
         * @tparam Unit the type of the units flag (inferred automatically)
         **/
        template<typename Unit>
        Euler(double a, double b, double c, Unit u);

        /**
         * Constructor to convert from a unit Quaternion.
         * The unit-ness of the Quaternion is not verified. If a non-unit
         * Quaternion is passed in, behaviour is undefined.
         *
         * @param quat the Quaternion to convert. Must be a unit quaternion,
         *             or behavior is undefined.
         **/
        explicit Euler(const Quaternion &quat);

        /**
         * Constructor to convert from a different Euler convention.
         *
         * @param o the other Euler object
         **/
        template<typename A2, typename B2, typename C2, typename Conv2>
        explicit Euler(const Euler<A2, B2, C2, Conv2> &o);

        /**
         * Constructor to convert from a Angular (or AngularVector)
         *
         * @param r the angular
         **/
        explicit Euler(const AngularVector &r);

        /// Getter for the first angular angle, around axis A
        double a() const { return a_; }
        /// Getter for the first angular angle, around axis B
        double b() const { return b_; }
        /// Getter for the first angular angle, around axis C
        double c() const { return c_; }

        /// Setter for the first angular angle, around axis A
        void a(double n) { a_ = n; }
        /// Setter for the first angular angle, around axis B
        void b(double n) { b_ = n; }
        /// Setter for the first angular angle, around axis C
        void c(double n) { c_ = n; }

        /**
         * Convert this Euler angle to a Quaternion
         *
         * @return the Quaternion which represents the same angle as *this
         **/
        Quaternion to_quat() const;

        /**
         * Convert this Euler angle to a Orientation (axis-angle notation),
         * within the default frame.
         *
         * @return the Orientation which represents the same angle as *this
         **/
        Orientation to_orientation() const;

        /**
         * Convert this Euler angle to an Orientation (axis-angle notation),
         * within the specified frame.
         *
         * @param frame the reference frame the Orientation will belong to
         * @return the Orientation which represents the same angle as *this
         **/
        Orientation to_orientation(const ReferenceFrame &frame) const;

        /**
         * Convert a Quaternion into a Euler angle.
         *
         * @param quat the Quaternion to convert
         * @return a Euler angle equivalent to quat
         **/
        static Euler from_quat(const Quaternion &quat);
      private:
        double a_, b_, c_;

        void set_from_quat(const Quaternion &quat);
      };

      typedef Euler<conv::X, conv::Y, conv::Z>             EulerXYZ;
      typedef Euler<conv::Y, conv::X, conv::Z>             EulerYXZ;
      typedef Euler<conv::X, conv::Y, conv::Z, conv::Intr> EulerIntrXYZ;
      typedef Euler<conv::Y, conv::X, conv::Z, conv::Intr> EulerIntrYXZ;
      typedef Euler<conv::X, conv::Y, conv::Z, conv::Extr> EulerExtrXYZ;
      typedef Euler<conv::Y, conv::X, conv::Z, conv::Extr> EulerExtrYXZ;
      typedef Euler<conv::Z, conv::Y, conv::X>             EulerZYX;
      typedef Euler<conv::Z, conv::Y, conv::X, conv::Intr> EulerIntrZYX;
      typedef Euler<conv::Z, conv::Y, conv::X, conv::Extr> EulerExtrZYX;

      /** The Euler convention used by VREP */
      typedef EulerIntrXYZ EulerVREP;

      /** A commonly used Euler convention: Yaw-Pitch-Roll */
      typedef EulerIntrZYX EulerYPR;

      /** A commonly used Euler convention: Yaw-Pitch-Roll */
      typedef EulerYPR YawPitchRoll;

      /** The most common vernacular usage of roll, pitch and yaw */
      typedef EulerExtrXYZ RollPitchYaw;

      /** Stream operator for Euler angles */
      template<typename A, typename B, typename C, typename Conv>
      std::ostream &operator<<(std::ostream &o, const Euler<A, B, C, Conv> &e);
    }
  }
}

#include "Euler.inl"

#endif
