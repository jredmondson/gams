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
 * angles (a sequence of orientations about specified axes). To manipulate
 * these angles, convert them to the Quaternion or Orientation classes.
 **/

#ifndef _GAMS_UTILITY_EULER_H_
#define _GAMS_UTILITY_EULER_H_

#include <gams/pose/Euler.h>
#include <gams/utility/AngleUnits.h>
#include <gams/utility/Orientation.h>
#include <gams/utility/Quaternion.h>

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
        typedef gams::pose::euler::conv::Intr Intr;
        typedef gams::pose::euler::conv::Extr Extr;
        typedef gams::pose::euler::conv::X X;
        typedef gams::pose::euler::conv::Y Y;
        typedef gams::pose::euler::conv::Z Z;
      } // namespace conv
    }
  }
}

namespace gams
{
  namespace utility
  {
    namespace euler
    {
      /**
       * Class template for representing an angle in Euler notation. Euler
       * notation represents an angle as a sequence of orientations about axes
       * applied in an order. The order of orientations, and whether they are
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
       * a Quaternion (or Orientation) and back, than to try to directly manipulate
       * a Euler angle.
       *
       * @tparam A the first axis of orientation
       * @tparam B the second axis of orientation
       * @tparam C the third axis of orientation
       * @tparam Conv whether to use Extrinsic (conv::Extr) or Intrisic
       *              (conv::Intr) orientations in this convention. Default is Intr
       **/
      template<typename A, typename B, typename C, typename Conv = conv::Intr>
      using Euler = gams::pose::euler::Euler<A, B, C, Conv>;

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
    }
  }
}

#endif
