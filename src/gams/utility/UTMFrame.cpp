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
 * @file UTMFrame.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the UTM reference frame class
 **/

#ifdef GAMS_UTM

#include "UTMFrame.h"
#include "GPSFrame.h"
#include "Quaternion.h"

#include <GeographicLib/UTMUPS.hpp>

using namespace gams::utility;

using namespace GeographicLib;

char UTMFrame::nato_band(double x, double y)
{
  if(y > MAX_Y)
  {
    int zone = to_zone(x);
    if(zone <= 30)
      return 'Y';
    else
      return 'Z';
  }
  else if(y < MIN_Y)
  {
    int zone = to_zone(x);
    if(zone <= 30)
      return 'A';
    else
      return 'B';
  }
  else
  {
    y = (y - MIN_Y) / (MAX_Y - MIN_Y);
    char ret =  'C' + int(y * 20);
    if(ret >= 'I') ++ret;
    if(ret >= 'O') ++ret;
    return ret;
  }
}

void UTMFrame::transform_location_to_origin(
                  double &x, double &y, double &z) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    UTMUPS::Reverse(to_zone(x), to_hemi(y),
                    to_easting(x), to_northing(y), y, x);
    return;
  }
  throw undefined_transform(*this, origin().frame(), true);
}

void UTMFrame::transform_location_from_origin(
                  double &x, double &y, double &z) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    int zone;
    bool hemi;
    double eas, nor;
    UTMUPS::Forward(y, x, zone, hemi, eas, nor, zone_);
    x = from_easting(eas, zone);
    y = from_northing(nor, hemi);
    return;
  }
  throw undefined_transform(*this, origin().frame(), true);
}

void UTMFrame::transform_rotation_to_origin(
                            double &rx, double &ry, double &rz) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    return;
  }
  throw undefined_transform(*this, origin().frame(), true);
}

void UTMFrame::transform_rotation_from_origin(
                            double &rx, double &ry, double &rz) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    return;
  }
  throw undefined_transform(*this, origin().frame(), false);
}

void UTMFrame::transform_pose_to_origin(
                double &x, double &y, double &z,
                double &rx, double &ry, double &rz) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    double gamma, k;
    UTMUPS::Reverse(to_zone(x), to_hemi(y),
                    to_easting(x), to_northing(y), y, x, gamma, k);
    Quaternion quat(rx, ry, rz);
    Quaternion gquat(0, 0, -DEG_TO_RAD(gamma));
    quat.pre_multiply(gquat);
    quat.to_rotation_vector(rx, ry, rz);
    return;
  }
  throw undefined_transform(*this, origin().frame(), true);
}

void UTMFrame::transform_pose_from_origin(
                double &x, double &y, double &z,
                double &rx, double &ry, double &rz) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    int zone;
    bool hemi;
    double eas, nor;
    double gamma, k;
    UTMUPS::Forward(y, x, zone, hemi, eas, nor, gamma, k, zone_);
    x = from_easting(eas, zone);
    y = from_northing(nor, hemi);
    Quaternion quat(rx, ry, rz);
    Quaternion gquat(0, 0, DEG_TO_RAD(gamma));
    quat.pre_multiply(gquat);
    quat.to_rotation_vector(rx, ry, rz);
    return;
  }
  throw undefined_transform(*this, origin().frame(), true);
}

double UTMFrame::calc_distance(
                  double x1, double y1, double z1,
                  double x2, double y2, double z2) const
{
  if(to_zone(x1) == to_zone(x2))
  {
    double x_dist = x2 - x1;
    double y_dist = y2 - y1;
    double z_dist = z2 - z2;

    return sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);
  }
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    Location l1(*this, x1, y1, z1);
    Location l2(*this, x2, y2, z2);

    l1.transform_this_to(*frame);
    l2.transform_this_to(*frame);

    return l1.distance_to(l2);
  }
  return NAN;
}

void UTMFrame::do_normalize_location(
                  double &x, double &y, double &z) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    if(zone_ < 0)
    {
      Location loc(*this, x, y, 0);
      loc.transform_this_to(*frame);
      loc.transform_this_to(*this);
      if(loc.zone() != to_zone(x) || loc.hemi() != to_hemi(y))
      {
        x = loc.x();
        y = loc.y();
      }
    }
  }
}

void UTMFrame::do_normalize_pose(
                double &x, double &y, double &z,
                double &rx, double &ry, double &rz) const
{
  GAMS_WITH_FRAME_TYPE(origin(), GPSFrame, frame)
  {
    if(zone_ < 0)
    {
      Pose pose(*this, x, y, 0, rx, ry, rz);
      pose.transform_this_to(*frame);
      pose.transform_this_to(*this);
      if(pose.zone() != to_zone(x) || pose.hemi() != to_hemi(y))
      {
        x = pose.x();
        y = pose.y();
        rx = pose.rx();
        ry = pose.ry();
        rz = pose.rz();
      }
    }
  }
}

#endif
