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
 * @file UTMFrame.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the base reference Frame class
 **/

#ifndef _GAMS_UTILITY_UTM_FRAME_H_
#define _GAMS_UTILITY_UTM_FRAME_H_

#ifdef GAMS_UTM

#include "ReferenceFrame.h"

namespace gams
{
  namespace utility
  {

    /**
     * A reference frame type  which handles UTM (between 84/-84 latitude)
     * and UPS coordinates (otherwise). All math is provided by GeographicLib
     * UTMUPS class; this is a wrapper to fit into the GAMS coordinate system
     * architecture.
     *
     * Be sure to use the northing(), easting(), zone(), and hemi() accessors
     * in Location. The base x() and y() values (especially x()) are not
     * traditional UTM representations.
     *
     * Distance calculations with UTMFrame are done two ways:
     *   If the two Locations are in the same zone, a simple cartesian distance
     *   is performed. Otherwise, if this frame is a child to a GPSFrame, the
     *   Locations are converted to that frame, then distance measured according
     *   to the GPSFrame.
     **/
    class GAMSExport UTMFrame : public AxisAngleFrame
    {
    public:
      /**
       * Constructs a new UTMFrame, not embedded in any frame
       *
       * @param zone the UTM zone to apply conversions within. If zet to -1
       *             (the default), all conversions will be done within the
       *             standard zone for the given latitude/longitude. This is
       *             the same as STANDARD pseudo-zone for GeographicLib::UTMUPS.
       *             The other pseudo-zones supported by that class can also be
       *             used here.
       **/
      UTMFrame(int zone = -1)
        : AxisAngleFrame(), zone_(zone) {}

      /**
       * Constructs a new UTMFrame, which must be embedded in a GPS frame
       *
       * @param parent the parent frame. Must be a GPS frame, of Earth.
       *               Behavior is undefined if not.
       * @param zone the UTM zone to apply conversions within. If zet to -1
       *             (the default), all conversions will be done within the
       *             standard zone for the given latitude/longitude. This is
       *             the same as STANDARD pseudo-zone for GeographicLib::UTMUPS.
       *             The other pseudo-zones supported by that class can also be
       *             used here.
       **/
      UTMFrame(const ReferenceFrame &parent, int zone = -1)
        : AxisAngleFrame(Pose(parent, 0, 0)), zone_(zone) {}

      int zone() { return zone_; }

      void zone(int zone) { zone_ = zone; }

    protected:

      static const double ZONE_WIDTH;
      static const double SOUTH_OFFSET;
      static const double MAX_Y;
      static const double MIN_Y;

      static constexpr int to_zone(double x)
      {
        return x < 0 ? 0 : 1 + int(x / ZONE_WIDTH);
      }

      static constexpr double to_easting(double x)
      {
        return x < 0 ? -x : x - ZONE_WIDTH * to_zone(x);
      }

      static constexpr int to_hemi(double y)
      {
        return y >= 0;
      }

      static constexpr int to_northing(double y)
      {
        return to_hemi(y) ? y : SOUTH_OFFSET + y;
      }

      static constexpr double from_easting(double e, int zone)
      {
        return (zone == 0) ? -e :
          e + (ZONE_WIDTH * ((zone - 1) % 60));
      }

      static constexpr double from_northing(double n, bool hemi)
      {
        return (hemi == true) ? n : n - SOUTH_OFFSET;
      }

      static char nato_band(double x, double y)
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
        }
      }

      /**
       * Returns the name of this type of reference frame.
       *
       * @return the string "UTM"
       **/
      virtual std::string get_name() const;

      /**
      * Transforms a location to origin
      * @param x   the x coordinate
      * @param y   the y coordinate
      * @param z   the z coordinate
      **/
      virtual void transform_location_to_origin(
                      double &x, double &y, double &z) const;

      /**
      * Transforms a location from origin
      * @param x   the x coordinate
      * @param y   the y coordinate
      * @param z   the z coordinate
      **/
      virtual void transform_location_from_origin(
                      double &x, double &y, double &z) const;

      /**
      * Calculates distance from one point to another
      * @param x1   the x coordinate of the first point
      * @param y1   the y coordinate of the first point
      * @param z1   the z coordinate of the first point
      * @param x2   the x coordinate of the second point
      * @param y2   the y coordinate of the second point
      * @param z2   the z coordinate of the second point
      **/
      virtual double calc_distance(
                      double x1, double y1, double z1,
                      double x2, double y2, double z2) const;

      /**
      * Normalizes a location
      * @param x   the x coordinate
      * @param y   the y coordinate
      * @param z   the z coordinate
      **/
      virtual void do_normalize_location(
                      double &x, double &y, double &z) const;

    private:
      int zone_;

      friend class LocationVector;
    };
  }
}

#include "UTMFrame.inl"

#endif
#endif
