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
 * @file Euler.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains inline functions of the Euler class
 **/

#ifndef _GAMS_POSE_EULER_INL_
#define _GAMS_POSE_EULER_INL_

#include <iostream>
#include <cmath>
#include "Euler.h"

namespace gams
{
  namespace pose
  {
    namespace euler
    {
      template<typename A, typename B, typename C, typename Conv>
      template<typename Unit>
      inline Euler<A,B,C,Conv>::Euler(double a, double b, double c, Unit u)
        : a_(u.to_radians(a)), b_(u.to_radians(b)), c_(u.to_radians(c)) {}

      template<typename A, typename B, typename C, typename Conv>
      inline Euler<A,B,C,Conv>::Euler(const Quaternion &quat)
        : a_(Conv::reverse ? F::eular_c(quat) : F::eular_a(quat)),
          b_(F::eular_b(quat)),
          c_(Conv::reverse ? F::eular_a(quat) : F::eular_c(quat)) {}

      template<typename A, typename B, typename C, typename Conv>
      void Euler<A,B,C,Conv>::set_from_quat(const Quaternion &quat)
      {
        a_ = Conv::reverse ? F::eular_c(quat) : F::eular_a(quat);
        b_ = F::eular_b(quat);
        c_ = Conv::reverse ? F::eular_a(quat) : F::eular_c(quat);
      }

      template<typename A, typename B, typename C, typename Conv>
      template<typename A2, typename B2, typename C2, typename Conv2>
      inline Euler<A,B,C,Conv>::Euler(const Euler<A2, B2, C2, Conv2> &o)
      {
        Quaternion quat(o.to_quat());
        set_from_quat(quat);
      }

      template<typename A, typename B, typename C, typename Conv>
      inline Euler<A,B,C,Conv>::Euler(const AngularVector &r)
      {
        Quaternion quat(r);
        set_from_quat(quat);
      }

      template<typename A, typename B, typename C, typename Conv>
      inline Quaternion Euler<A,B,C,Conv>::to_quat() const
      {
        // Precalculates and caches all sin/cos required by F::quat_*
        Trig trig(Conv::reverse ? c_ : a_, b_, Conv::reverse ? a_ : c_);
        return Quaternion(F::quat_x(trig), F::quat_y(trig),
                          F::quat_z(trig), F::quat_w(trig));
      }

      template<typename A, typename B, typename C, typename Conv>
      inline Orientation Euler<A,B,C,Conv>::to_orientation() const
      {
        return Orientation(to_quat());
      }

      template<typename A, typename B, typename C, typename Conv>
      inline Orientation Euler<A,B,C,Conv>::to_orientation(
        const ReferenceFrame &frame) const
      {
        return Orientation(frame, to_quat());
      }

      template<typename A, typename B, typename C, typename Conv>
      inline Euler<A,B,C,Conv>
      Euler<A,B,C,Conv>::from_quat(const Quaternion &quat)
      {
        return Euler(quat);
      }

      template<typename A, typename B, typename C, typename Conv>
      inline std::ostream &
      operator<<(std::ostream &o, const Euler<A, B, C, Conv> &e)
      {
        return o << "Euler" <<
                 Conv::name() << A::name() << B::name() << C::name() << "(" <<
                 e.a() << ", " << e.b() << ", " << e.c() << ")";
      }
    }
  }
}

#endif
