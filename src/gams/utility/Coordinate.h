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
 * @file Coordinate.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location, Orientation, and Pose classes
 **/

#ifndef _GAMS_UTILITY_COORDINATE_H_
#define _GAMS_UTILITY_COORDINATE_H_

#include "gams/pose/Coordinate.h"

namespace gams
{
  namespace utility
  {
    namespace order
    {
      /**
       * Used to specify ordering for reading from a container, e.g.,
       * as template argument to to_container and from_container.
       *
       * Use the typedefs below; they cover all possible instantiations
       * with readable names.
       **/
      template<int i0_, int i1_, int i2_>
      using Order = gams::pose::order::Order<i0_, i1_, i2_>;

      static const int X = 0;
      static const int Lng = X;

      static const int Y = 1;
      static const int Lat = Y;

      static const int Z = 2;
      static const int Alt = Z;

      /** Use to read/write in X/Y/Z order */
      typedef Order<X, Y, Z> XYZ;

      /** Use to read/write in X/Z/Y order */
      typedef Order<X, Z, Y> XZY;

      /** Use to read/write in Y/X/Z order */
      typedef Order<Y, X, Z> YXZ;

      /** Use to read/write in Y/Z/X order */
      typedef Order<Y, Z, X> YZX;
      
      /** Use to read/write in Z/X/Y order */
      typedef Order<Z, X, Y> ZXY;
      
      /** Use to read/write in Z/Y/X order */
      typedef Order<Z, Y, X> ZYX;

      /** Use to read/write in Lat/Lng/Alt order */
      typedef Order<Lat, Lng, Alt> LatLng;

      /** Use to read/write in Lng/Lat/Alt order */
      typedef Order<Lng, Lat, Alt> LngLat;

      /** Use to read in conventional GPS Lat/Lng/Alt order */
      typedef LatLng GPS;
    }

    /**
     * New coordinate types which are frame-dependant can inherit from this
     * class. Pass the type of the child class as CoordType
     *
     * New coordinate types must:
     *   -- Inherit from Coordinate, and pass itself as template parameter
     *   -- Inherit from a base class which does not inherit from Coordinate
     *   -- That base class must:
     *      -- Have a typedef BaseType which refers to itself
     *      -- Implement operator==
     *      -- Implement the following methods:
     *   static std::string name() // return the type's name
     *   static int size() // return # of values in representation
     *   double get(int i) const // return ith value of representation
     *   bool operator==(const BaseType &rhs) const
     *   BaseType &as_vec() // return *this
     *   const BaseType &as_vec() const // return *this
     *
     * Additionally, new coordinate types should either:
     *   -- Specialize the ReferenceFrame::*_within_frame templates; OR
     *   -- Add new overloads for transform_to_origin, transform_from_origin,
     *      do_normalize, and calc_distance virtual methods in ReferenceFrame,
     *      and add transformation logic for those methods in the various frames
     **/
    template<typename CoordType>
    using Coordinate = gams::pose::Coordinate<CoordType>;
  }
}

#endif
