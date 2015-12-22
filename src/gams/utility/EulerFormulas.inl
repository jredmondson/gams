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
 * @file EulerFormulas.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains implementation details for the Euler class template.
 * It currently implements intrinsic and extrinsice XYZ and ZYX euler
 * conventions. If others are required, specializations of the EulerFormulas
 * template must be added (a compile-time error will occur if one does not
 * exist). See below for details.
 **/

#ifndef _GAMS_UTILITY_EULER_FORMULAS_INL_
#define _GAMS_UTILITY_EULER_FORMULAS_INL_

#include <cmath>
#include "gams/utility/Quaternion.h"

#include "Euler.h"

namespace gams
{
  namespace utility
  {
    namespace euler
    {
      namespace detail
      {
        using namespace conv;

        template<typename A, typename B, typename C>
        struct EulerFormulas
        {
          static_assert(sizeof(A) == -1, "Formulas for this convention " \
            "not implemented. See src/gams/utility/EulerFormulas.inl"
          );
        };

        struct EulerTrigABC
        {
          EulerTrigABC(double a, double b, double c)
            : sin_a(sin(a / 2)), cos_a(cos(a / 2)),
              sin_b(sin(b / 2)), cos_b(cos(b / 2)),
              sin_c(sin(c / 2)), cos_c(cos(c / 2)) {}

          double sin_a, cos_a;
          double sin_b, cos_b;
          double sin_c, cos_c;
        };

        struct EulerTrigABA
        {
          EulerTrigABA(double a, double b, double c)
            : sin_b(sin(b / 2)), cos_b(cos(b / 2)),
              sin_apc(sin(a + c)), cos_apc(cos(a + c)),
              sin_amc(sin(a - c)), cos_amc(cos(a - c)) {}

          double sin_b, cos_b;
          double sin_apc, cos_apc;
          double sin_amc, cos_amc;
        };

        template<typename A, typename B, typename C>
        struct GetEulerTrig
        {
          typedef EulerTrigABC type;
        };

        template<typename A, typename B>
        struct GetEulerTrig<A, B, A>
        {
          typedef EulerTrigABA type;
        };

        template<typename A, typename B, typename C, typename Conv = Intr>
        struct GetTypes
        {
          typedef EulerFormulas<A, B, C> F;
          typedef typename GetEulerTrig<A, B, C>::type Trig;
        };

        template<typename A, typename B, typename C>
        struct GetTypes<A, B, C, Extr>
        {
          typedef EulerFormulas<C, B, A> F;
          typedef typename GetEulerTrig<C, B, A>::type Trig;
        };

        struct radians_t
        {
          double to_radians(double in) { return in; }
          double from_radians(double in) { return in; }
        };

        struct degrees_t
        {
          double to_radians(double in) { return DEG_TO_RAD(in); }
          double from_radians(double in) { return RAD_TO_DEG(in); }
        };

        struct revolutions_t
        {
          double to_radians(double in) { return in * 2 * M_PI; }
          double from_radians(double in) { return in / (2 * M_PI); }
        };

        // Conversion formulas from NASA document:
        // "Euler Angles, Quaternions and Transformation Matrices", July 1977
        // http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf

        template<>
        struct EulerFormulas<X, Y, Z>
        {
          typedef GetEulerTrig<X, Y, Z>::type Trig;

          static double eular_a(const Quaternion &q)
          {
            return atan2(-q.m23(), q.m33());
          };

          static double eular_b(const Quaternion &q)
          {
            return asin(q.m13());
          };

          static double eular_c(const Quaternion &q)
          {
            return atan2(-q.m12(), q.m11());
          };

          static double quat_w(const Trig &t)
          {
            return -t.sin_a * t.sin_b * t.sin_c + t.cos_a * t.cos_b * t.cos_c;
          };

          static double quat_x(const Trig &t)
          {
            return t.sin_a * t.cos_b * t.cos_c + t.sin_b * t.sin_c * t.cos_a;
          };

          static double quat_y(const Trig &t)
          {
            return -t.sin_a * t.sin_c * t.cos_b + t.sin_b * t.cos_a * t.cos_c;
          };

          static double quat_z(const Trig &t)
          {
            return t.sin_a * t.sin_b * t.cos_c + t.sin_c * t.cos_a * t.cos_b;
          };
        };

        template<>
        struct EulerFormulas<Z, Y, X>
        {
          typedef GetEulerTrig<Z, Y, X>::type Trig;

          static double eular_a(const Quaternion &q)
          {
            return atan2(q.m21(), q.m11());
          };

          static double eular_b(const Quaternion &q)
          {
            return asin(-q.m31());
          };

          static double eular_c(const Quaternion &q)
          {
            return atan2(q.m32(), q.m33());
          };

          static double quat_w(const Trig &t)
          {
            return t.sin_a * t.sin_b * t.sin_c + t.cos_a * t.cos_b * t.cos_c;
          };

          static double quat_x(const Trig &t)
          {
            return -t.sin_a * t.sin_b * t.cos_c + t.sin_c * t.cos_a * t.cos_b;
          };

          static double quat_y(const Trig &t)
          {
            return t.sin_a * t.sin_c * t.cos_b + t.sin_b * t.cos_a * t.cos_c;
          };

          static double quat_z(const Trig &t)
          {
            return t.sin_a * t.cos_b * t.cos_c - t.sin_b * t.sin_c * t.cos_a;
          };
        };

        // Add any new specializations of EulerFormulas here, as needed. See
        // the NASA document linked above for the necessary formulas, and
        // follow the pattern established by the existing specializations.
        // Note that extrinsic and instrinsic conventions use the same formulas,
        // just in reverse order of arguments, so if you specify some ordering
        // ABC, you should also specify CBA.

      } // namespace detail
    }
  }
}

#endif
