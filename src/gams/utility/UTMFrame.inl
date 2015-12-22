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

#ifndef _GAMS_UTILITY_GPS_FRAME_INL_
#define _GAMS_UTILITY_GPS_FRAME_INL_

#ifdef GAMS_UTM

#include "ReferenceFrame.h"

#include "UTMFrame.h"

namespace gams
{
  namespace utility
  {
    inline std::string UTMFrame::get_name() const
    {
      return "UTM";
    }

    /**
     * Gets the UTM northing for this location.
     * Will return arbitrary value if this location is not bound to a UTMFrame
     **/
    inline constexpr double LocationVector::northing() const
    {
      return UTMFrame::to_northing(y_);
    }

    /**
     * Gets the UTM easting for this location.
     * Will return arbitrary value if this location is not bound to a UTMFrame
     **/
    inline constexpr double LocationVector::easting() const
    {
      return UTMFrame::to_easting(x_);
    }

    /**
     * Gets the UTM zone (longitudinal) for this location: 0 if USP (near
     * poles) otherwise in range [1, 60]
     * Will return arbitrary value if this location is not bound to a UTMFrame
     **/
    inline constexpr int LocationVector::zone() const
    {
      return UTMFrame::to_zone(x_);
    }

    /**
     * Gets the UTM hemisphere for this location. True if northern.
     * Will return arbitrary value if this location is not bound to a UTMFrame
     **/
    inline constexpr bool LocationVector::hemi() const
    {
      return UTMFrame::to_hemi(x_);
    }

    /**
     * Set UTM northing and hemisphere (true is northern) simultaneously
     *
     * @param n the new northing
     * @param h the new hemisphere
     **/
    inline void LocationVector::northing(double n, bool h)
    {
      y_ = UTMFrame::from_northing(n, h);
    }

    /**
     * Set UTM northing only; do not change hemisphere
     *
     * @param n the new northing
     **/
    inline void LocationVector::northing(double n)
    {
      northing(n, hemi());
    }

    /**
     * Set UTM hemisphere (true is northern) only; do not change northing
     *
     * @param h the new hemisphere
     **/
    inline void LocationVector::hemi(bool h)
    {
      northing(northing(), h);
    }

    /**
     * Set UTM easting and zone simultaneously
     *
     * @param e the new easting
     * @param z the new zone; 0 for USP (polar regions), otherwise [1,60]
     **/
    inline void LocationVector::easting(double e, int z)
    {
      x_ = UTMFrame::from_easting(e, z);
    }

    /**
     * Set UTM easting only; do not change zone
     *
     * @param e the new easting
     **/
    inline void LocationVector::easting(double e)
    {
      easting(e, zone());
    }

    /**
     * Set UTM zone only; do not change easting
     *
     * @param z the new zone; 0 for USP (polar regions), otherwise [1,60]
     **/
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
