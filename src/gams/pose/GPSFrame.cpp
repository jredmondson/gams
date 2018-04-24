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
 * @file GPSFrame.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the GPS reference frame class
 **/

#include "GPSFrame.h"

namespace gams
{
  namespace pose
  {
    namespace gps {
      void transform_linear_to_origin(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z)
      {
        throw undefined_transform(self, origin, true);
      }

      void transform_linear_from_origin(
                      const ReferenceFrameType *origin,
                      const ReferenceFrameType *self,
                      double ox, double oy, double oz,
                      double orx, double ory, double orz,
                      double &x, double &y, double &z)
      {
        throw undefined_transform(self, origin, false);
      }

      double calc_distance(
                const ReferenceFrameType *,
                double x1, double y1, double z1,
                double x2, double y2, double z2)
      {
        double alt = z1;
        double alt_diff = z2 - z1;
        if(z2 < alt)
          alt = z2;

        /**
         * Calculate great circle distance using numerically stable formula from
         * http://en.wikipedia.org/w/index.php?title=Great-circle_distance&oldid=659855779
         * Second formula in "Computational formulas"
         **/
        double lat1 = DEG_TO_RAD(y1);
        double lng1 = DEG_TO_RAD(x1);
        double lat2 = DEG_TO_RAD(y2);
        double lng2 = DEG_TO_RAD(x2);

        double sin_lat1 = sin(lat1);
        double sin_lat2 = sin(lat2);
        double cos_lat1 = cos(lat1);
        double cos_lat2 = cos(lat2);

        double delta_lng = lng2 - lng1;
        if(delta_lng < 0)
          delta_lng = -delta_lng;

        double sin_delta_lng = sin(delta_lng);
        double cos_delta_lng = cos(delta_lng);

        double top_first = cos_lat2 * sin_delta_lng;
        double top_second =
            cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_delta_lng;

        double top = sqrt(top_first * top_first + top_second * top_second);

        double bottom = sin_lat1 * cos_lat2 + cos_lat1 * cos_lat2 * cos_delta_lng;

        const double epsilon = 0.000001;
        /**
         * atan2(0, 0) is undefined, but for our purposes, we can treat it as 0
         **/
        double central_angle =
            (fabs(top) < epsilon && fabs(bottom) < epsilon)
                 ? 0 : atan2(top, bottom);

        double great_circle_dist = (EARTH_RADIUS + alt) * central_angle;

        if(alt_diff == 0)
          return great_circle_dist;
        else
          return sqrt(great_circle_dist * great_circle_dist + alt_diff*alt_diff);
      }

      void normalize_linear(
                const ReferenceFrameType *,
                double &x, double &y, double &z)
      {
        while(y > 90.000001)
        {
          y -= 180;
          x += 180;
        }
        while(y < -90.000001)
        {
          y += 180;
          x -= 180;
        }
        while(x > 180.000001)
          x -= 180;
        while(x < -180.000001)
          x += 180;
      }

      const ReferenceFrameType GPSImpl = {
        2, "GPS",
        transform_linear_to_origin,
        transform_linear_from_origin,
        normalize_linear,
        calc_distance,
        simple_rotate::transform_angular_to_origin,
        simple_rotate::transform_angular_from_origin,
        default_normalize_angular,
        simple_rotate::calc_angle,
        simple_rotate::transform_pose_to_origin,
        simple_rotate::transform_pose_from_origin,
        default_normalize_pose,
      };
    }

    const ReferenceFrameType *GPS = &gps::GPSImpl;

    const ReferenceFrame &gps_frame (void) {
      static ReferenceFrame frame{GPS, "gps_frame", Pose(ReferenceFrame(), 0, 0)};
      return frame;
    }
  }
}
