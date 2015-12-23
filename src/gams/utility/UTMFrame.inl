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
 * @file GPSFrame.inl
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_UTM_FRAME_INL_
#define _GAMS_UTILITY_UTM_FRAME_INL_

#ifdef GAMS_UTM

#include "ReferenceFrame.h"

#include "UTMFrame.h"

namespace gams
{
  namespace utility
  {
    inline UTMFrame::UTMFrame(int zone)
        : ReferenceFrame(), zone_(zone) {}

    inline UTMFrame::UTMFrame(const ReferenceFrame &parent, int zone)
      : ReferenceFrame(Pose(parent, 0, 0)), zone_(zone) {}

    inline int UTMFrame::zone() { return zone_; }

    inline void UTMFrame::zone(int zone) { zone_ = zone; }

    inline Location UTMFrame::mk_loc(double easting, int zone,
                                     double northing, bool hemi)
    {
      return Location(*this, from_easting(easting, zone),
                      from_northing(northing, hemi));
    }

    inline std::string UTMFrame::get_name() const
    {
      return "UTM";
    }

    inline constexpr int UTMFrame::to_zone(double x)
    {
      return x < 0 ? 0 : 1 + int(x / ZONE_WIDTH);
    }

    inline constexpr double UTMFrame::to_easting(double x)
    {
      return x < 0 ? x + UPS_OFFSET : x - ZONE_WIDTH * (to_zone(x) - 1);
    }

    inline constexpr bool UTMFrame::to_hemi(double y)
    {
      return y >= 0;
    }

    inline constexpr double UTMFrame::to_northing(double y)
    {
      return to_hemi(y) ? y : SOUTH_OFFSET + y;
    }

    inline constexpr double UTMFrame::from_easting(double e, int zone)
    {
      return zone == 0 ? e - UPS_OFFSET : e + (ZONE_WIDTH * ((zone - 1) % 60));
    }

    inline constexpr double UTMFrame::from_northing(double n, bool hemi)
    {
      return hemi == true ? n : n - SOUTH_OFFSET;
    }

    inline constexpr double LocationVector::northing() const
    {
      return UTMFrame::to_northing(y_);
    }

    inline constexpr double LocationVector::easting() const
    {
      return UTMFrame::to_easting(x_);
    }

    inline constexpr int LocationVector::zone() const
    {
      return UTMFrame::to_zone(x_);
    }

    inline constexpr bool LocationVector::hemi() const
    {
      return UTMFrame::to_hemi(y_);
    }

    inline void LocationVector::northing(double n, bool h)
    {
      y_ = UTMFrame::from_northing(n, h);
    }

    inline void LocationVector::northing(double n)
    {
      northing(n, hemi());
    }

    inline void LocationVector::hemi(bool h)
    {
      northing(northing(), h);
    }

    inline void LocationVector::easting(double e, int z)
    {
      x_ = UTMFrame::from_easting(e, z);
    }

    inline void LocationVector::easting(double e)
    {
      easting(e, zone());
    }

    inline void LocationVector::zone(int z)
    {
      easting(easting(), z);
    }

    inline char LocationVector::nato_band()
    {
      return UTMFrame::nato_band(x_,y_);
    }
  }
}

#endif
#endif
