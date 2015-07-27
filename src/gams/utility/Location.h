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
 * @file Location.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location class
 **/

#ifndef _GAMS_UTILITY_LOCATION_H_
#define _GAMS_UTILITY_LOCATION_H_

#include <iostream>
#include <string>

#include <stdexcept>
#include <gams/utility/Coordinate.h>

namespace gams
{
  namespace utility
  {
    class Reference_Frame;

    /**
     * Container for Location information, not bound to a frame.
     * Stores a 3-tuple, for x, y, and z.
     *
     * Provides accessor methods to support non-cartesian coordinate systems:
     *
     * lng/lat/alt for GPS-style systems
     * rho/phi/r for Cylindrical systems
     * theta/phi/r for Spherical systems
     *
     * Each of the above are bound to x/y/z respectively.
     **/
    class Location_Vector
    {
    public:
      /**
       * Primary constructor
       *
       * @param x the x coordinate of the new Location
       * @param y the y coordinate of the new Location
       * @param z the z coordinate of the new Location; defaults to zero
       **/
      constexpr Location_Vector(double x, double y, double z = 0.0);

      /**
       * Default constructor. Initializes an invalid Location (INVAL_COORD).
       **/
      constexpr Location_Vector();

      /**
       * Copy constructor.
       **/
      constexpr Location_Vector(const Location_Vector &orig);

      /**
       * Tests if this Location is valid
       *
       * @return true if no values in this Location are INVAL_COORD
       **/
      constexpr bool is_invalid() const;

      /**
       * Tests if all values in this Location are the same
       * as the ones in the rhs.
       *
       * @param rhs the other Location to check against
       **/
      constexpr bool operator==(const Location_Vector &rhs) const;

      /**
       * Tests if all values in this Location are zero
       *
       * @return true of all values are zero
       **/
      constexpr bool is_zero() const;

      /**
       * Returns the name of this coordinate type
       *
       * @return "Location"
       **/
      static std::string name();

      /**
       * Getter for x
       *
       * @return x value
       **/
      constexpr double x() const;

      /**
       * Getter for y
       *
       * @return y value
       **/
      constexpr double y() const;

      /**
       * Getter for z
       *
       * @return z value
       **/
      constexpr double z() const;

      /**
       * Setter for x
       *
       * @param new_x the new x value
       * @return new x value
       **/
      double x(double new_x);

      /**
       * Setter for y
       *
       * @param new_y the new y value
       * @return new y value
       **/
      double y(double new_y);

      /**
       * Setter for z
       *
       * @param new_z the new z value
       * @return new z value
       **/
      double z(double new_z);

      /**
       * Getter for longitude, a synonym for x
       *
       * @return longitude value
       **/
      constexpr double lng() const;

      /**
       * Getter for latitude, a synonym for y
       *
       * @return latitude value
       **/
      constexpr double lat() const;

      /**
       * Getter for altitude, a synonym for z
       *
       * @return altitude value
       **/
      constexpr double alt() const;

      /**
       * Setter for longitude, a synonym for x
       *
       * @param new_lng new longitude value
       * @return new longitude value
       **/
      double lng(double new_lng);

      /**
       * Setter for latitude, a synonym for y
       *
       * @param new_lat new latitude value
       * @return new latitude value
       **/
      double lat(double new_lat);

      /**
       * Setter for altitude, a synonym for z
       *
       * @param new_alt new altitude value
       * @return new altitude value
       **/
      double alt(double new_alt);

      /**
       * Getter for rho, a synonym for x for Cylindrical systems
       *
       * @return rho value
       **/
      constexpr double rho() const;

      /**
       * Getter for theta, a synonym for x for Spherical systems
       *
       * @return theta value
       **/
      constexpr double theta() const;

      /**
       * Getter for phi, a synonym for y for Cylindrical and Spherical systems
       *
       * @return phi value
       **/
      constexpr double phi() const;

      /**
       * Getter for r, a synonym for z for Cylindrical and Spherical systems
       *
       * @return r value
       **/
      constexpr double r() const;

      /**
       * Setter for rho, a synonym for x for Cylindrical systems
       *
       * @param new_rho new rho value
       * @return new rho value
       **/
      double rho(double new_rho);

      /**
       * Setter for theta, a synonym for x for Spherical systems
       *
       * @param new_theta new theta value
       * @return new theta value
       **/
      double theta(double new_theta);

      /**
       * Setter for phi, a synonym for y for Cylindrical and Spherical systems
       *
       * @param new_phi new phi value
       * @return new phi value
       **/
      double phi(double new_phi);

      /**
       * Setter for r, a synonym for z for Cylindrical and Spherical systems
       *
       * @param new_r new new value
       * @return new r value
       **/
      double r(double new_r);

      /**
       * Number of elements in this tuple
       *
       * @return the integer 3
       **/
      constexpr static int size();

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z
       *
       * @param i the index
       * @return the i'th value
       **/
      constexpr double get(int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z
       *
       * @param i the index to set
       * @param val the value to set to
       **/
      double set(int i, double val);

      typedef Location_Vector Base_Type;

      /**
       * Returns reference to this as the above Base_Type; useful
       * for derived types
       *
       * @return reference to this as Base_Type
       **/
      Base_Type &as_vec();

      /**
       * Returns const reference to this as the above Base_Type; useful
       * for derived types
       *
       * @return const reference to this as Base_Type
       **/
      constexpr const Base_Type &as_vec() const;

      friend class Quaternion;

      friend class Reference_Frame;

    private:
      double x_, y_, z_;
    };

    /**
     * Represents a Location within a reference frame.
     * This location always has x, y, and z coordinates, but interpretation
     * of those coordinates can vary according to the reference frame.
     *
     * Provides accessor methods to support non-cartesian coordinate systems:
     *
     * lng/lat/alt for GPS-style systems
     * rho/phi/r for Cylindrical systems
     * theta/phi/r for Spherical systems
     *
     * Each of the above are bound to x/y/z respectively.
     **/
    class Location : public Location_Vector, public Coordinate<Location>
    {
    public:
      /**
       * Primary constructor, using default reference frame.
       *
       * @param x the x coordinate of the new Location
       * @param y the y coordinate of the new Location
       * @param z the z coordinate of the new Location; defaults to zero
       **/
      Location(double x, double y, double z = 0.0);

      /**
       * Primary constructor
       *
       * @param frame the reference frame to bind to. This object must not
       *    outlive this Reference_Frame object.
       * @param x the x coordinate of the new Location
       * @param y the y coordinate of the new Location
       * @param z the z coordinate of the new Location; defaults to zero
       **/
      constexpr Location(const Reference_Frame &frame,
                         double x = 0.0, double y = 0.0, double z = 0.0);

      /**
       * Default constructor. Initializes an invalid Location (INVAL_COORD).
       **/
      Location();

      /**
       * Copy constructor.
       **/
      constexpr Location(const Location &orig);

      /**
       * Copy constructor, but transform into the new frame as well.
       *
       * @param new_frame the new frame to transform to
       * @param orig      the origin location
       **/
      Location(const Reference_Frame &new_frame, const Location &orig);

      using Coordinate<Location>::operator==;
    };
  }
}

#include "Location.inl"

// Include if not already included
#include <gams/utility/Pose.h>

#endif
