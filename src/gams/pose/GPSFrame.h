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
 * @file GPSFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#include "ReferenceFrame.h"

#ifndef _GAMS_POSE_GPS_FRAME_H_
#define _GAMS_POSE_GPS_FRAME_H_

#include "ReferenceFrame.h"

namespace gams
{
  namespace pose
  {
    /**
     * Contains functions for translating GPS frames.
     *
     * In GPS frames, positions are represented as GPS coordinates, and an
     * altitude, assuming a spherical Earth:
     *    x is Latitude
     *    y is Longitude
     *    z is Altitude (above assumed perfectly spherical surface)
     * Orientations represented in Axis Angle notation
     *    Axis rx points towards north pole
     *    Axis ry points east at current position
     *    Axis rz points downwards (i.e, reverse normal vector)
     *
     * Note that under this scheme, change in x and/or y position, while
     *   maintaining the same orientation angles, implies a rotation relative
     *   to the planet.
     *
     * Distances at same altitude calculated as distance along great circle of
     * sphere of radius EARTH_RADIUS plus altitude. If altitude differs,
     * great circle distance first calculated using lower of two altitudes,
     * then distance calculated as follows:
     *    distance = sqrt(circle_distance ^ 2 + altitude_difference ^ 2)
     *
     * This frame can have cartesian frames embedded within it, but cannot
     * be embedded within any other frames at this time.
     **/
    namespace gps {
    }

    constexpr double EARTH_RADIUS = 6371000.0;
    constexpr double EARTH_CIRC = EARTH_RADIUS * 2 * M_PI;

    /**
     * ReferenceFrameType for GPS frames. Pass as first argument of
     * ReferenceFrame constructors to create a GPS frame instead of the
     * default Cartesian; however, you should generally not create new
     * GPS frames. Instead, use gps_frame().
     **/
    extern GAMS_EXPORT const ReferenceFrameType *GPS;

    /**
     * Returns the canonical GPS frame representing Earth. In general, use
     * this function, instead of creating your own GPS typed ReferenceFrame.
     **/
    GAMS_EXPORT const ReferenceFrame &gps_frame(void);
  }
}

#include "GPSFrame.inl"
#endif

