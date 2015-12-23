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

#include "CartesianFrame.h"

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
     *   to the GPSFrame. If neither of the above apply, distance_to is NAN
     *
     * Note: UTM's notion of bearing is not the same as GPS; the y axis does not
     *       necessarily point in the north direction, especially for near-polar
     *       UPS coordinates. For proper transformation of poses, be sure to
     *       transform the entire pose as a single Pose object. Do not transform
     *       Locations and Rotations individually if bearing is important.
     **/
    class GAMSExport UTMFrame : public ReferenceFrame
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
      UTMFrame(int zone = -1);

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
      UTMFrame(const ReferenceFrame &parent, int zone = -1);

      /**
       * Get zone ID (or GeographicLib::UTMUPS pseudo-zone)
       *
       * @return the zone ID
       **/
      int zone();

      /**
       * Set zone ID (or GeographicLib::UTMUPS pseudo-zone)
       *
       * @param zone the zone ID
       **/
      void zone(int zone);

      /**
       * Creates a Location in this reference frame with the given UTM
       * parameters.
       *
       * @return a newly created Location
       **/
      Location mk_loc(double easting, int zone, double northing, bool hemi);

    protected:

      static const int KM = 1000;
      static const int ZONE_WIDTH = 1000 * KM;
      static const int SOUTH_OFFSET = 10000 * KM;
      static const int UPS_OFFSET = 4000 * KM;
      static const int MAX_Y = 9600 * KM;
      static const int MIN_Y = -9000 * KM;

      static constexpr int to_zone(double x);

      static constexpr double to_easting(double x);

      static constexpr bool to_hemi(double y);

      static constexpr double to_northing(double y);

      static constexpr double from_easting(double e, int zone);

      static constexpr double from_northing(double n, bool hemi);

      static char nato_band(double x, double y);

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
       * Transform RotationVector in-place into its origin frame from this frame
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_rotation_to_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform RotationVector in-place from its origin frame
       *
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_rotation_from_origin(
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform pose in-place into its origin frame from this frame.
       * Rotations may transform differenly based on Location, so transformation
       * of Poses might result in different values than transformation of the
       * Location and Rotation parts separately.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_to_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

      /**
       * Transform pose in-place from its origin frame
       * Rotations may transform differenly based on Location, so transformation
       * of Poses might result in different values than transformation of the
       * Location and Rotation parts separately.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx  the x component of the axis-angle representation
       * @param ry  the y component of the axis-angle representation
       * @param rz  the z component of the axis-angle representation
       **/
      virtual void transform_pose_from_origin(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

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
       * Normalizes a Location; if zone is -1, ensure that the location is in
       * the standard zone.
       * @param x   the x coordinate
       * @param y   the y coordinate
       * @param z   the z coordinate
       **/
      virtual void do_normalize_location(
                      double &x, double &y, double &z) const;

      /**
       * Normalizes a Pose; if zone is -1, ensure that the location is in
       * the standard zone. If moving to a different zone is required, updates
       * the rotation part accordingly as well.
       *
       * @param x the x axis for the coordinate to translate
       * @param y the y axis for the coordinate to translate
       * @param z the z axis for the coordinate to translate
       * @param rx   x axis of rotation
       * @param ry   y axis of rotation
       * @param rz   z axis of rotation
       **/
      virtual void do_normalize_pose(
                      double &x, double &y, double &z,
                      double &rx, double &ry, double &rz) const;

    private:
      int zone_;

      friend class LocationVector;
    };
  }
}

#include "UTMFrame.inl"

#endif
#endif
