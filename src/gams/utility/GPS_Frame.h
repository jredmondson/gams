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
 * @file Base_Frame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_GPS_FRAME_H_
#define _GAMS_UTILITY_GPS_FRAME_H_

#include "Reference_Frame.h"

namespace gams
{
  namespace utility
  {
    /**
     * Locations represented as GPS coordinates, and an altitude,
     * assuming a spherical planet (by default, Earth):
     *    x is Latitude
     *    y is Longitude
     *    z is Altitude (above assumed perfectly spherical surface)
     * Rotations represented as Euler angles, applied in this order:
     *    alpha degrees around axis pointing towards north pole at current location
     *      At north pole: points towards 0 Longitude; at south pole, towards 180 Longitude
     *    beta degrees around axis pointing west at current location
     *      At north and south poles: axis points towards 90W Longitude
     *    gamma degrees around axis pointing upwards (i.e, normal vector) at current location
     *
     * Note that under this scheme, change in x and/or y position, while maintaining the
     *   same rotation angles, implies a rotation relative to the planet.
     *
     * Distances at same altitude calculated as distance along great circle of sphere of
     * radius planet_radius plus altitude. If altitude differs, great circle distance first
     * calculated using lower of two altitudes, then distance calculated as follows:
     *    distance = sqrt(circle_distance ^ 2 + altitude_difference ^ 2)
     **/
    class GAMS_Export GPS_Frame : public Axis_Angle_Frame
    {
    public:
      static const double EARTH_RADIUS;
      static const double MOON_RADIUS;
      static const double MARS_RADIUS;

      GPS_Frame(double planet_radius = EARTH_RADIUS)
        : Axis_Angle_Frame(), _planet_radius(planet_radius) {}

      explicit GPS_Frame(const Pose &origin, double planet_radius = EARTH_RADIUS)
        : Axis_Angle_Frame(origin), _planet_radius(planet_radius) {}

      explicit GPS_Frame(Pose *origin, double planet_radius = EARTH_RADIUS)
        : Axis_Angle_Frame(origin), _planet_radius(planet_radius) {}

      double radius() const { return _planet_radius; }
      double circ() const { return 2 * _planet_radius * M_PI; }

      double radius(double new_radius) { return _planet_radius = new_radius; }
      double circ(double new_circ)
      {
        return _planet_radius = new_circ / (2 * M_PI);
      }

    private:
      double _planet_radius;

      virtual std::string get_name() const { return "GPS"; }

      virtual void transform_to_origin(Location_Base &in) const;

      virtual void transform_from_origin(Location_Base &in) const;

      virtual double calc_distance(const Location_Base &loc1, const Location_Base &loc2) const;

      virtual void do_normalize(Location_Base &in) const;
    };
  }
}

#endif
