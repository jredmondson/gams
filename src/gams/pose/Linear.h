/**
 * Copyright (c) 2018 Carnegie Mellon University. All Rights Reserved.
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
 *      are those of the author (s) and do not necessarily reflect the views of
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
 * @file Linear.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Linear and LinearVector classes
 **/

#ifndef _GAMS_POSE_LINEAR_H_
#define _GAMS_POSE_LINEAR_H_

#include <iostream>
#include <string>

#include <stdexcept>
#include <gams/pose/Coordinate.h>
#include <madara/knowledge/containers/DoubleVector.h>
#include <madara/knowledge/containers/NativeDoubleVector.h>

namespace gams
{
  namespace pose
  {
    class ReferenceFrame;

    /**
     * Container for Linear information, not bound to a frame.
     * Stores a 3-tuple, for x, y, and z.
     *
     * Provides accessor methods to support non-cartesian coordinate systems:
     *
     * lng/lat/alt for GPS-style systems
     * rho/phi/r for Cylindrical systems
     * theta/phi/r for Spherical systems
     * northing/easting/zone/hemi/alt for UTM/USP systems
     *
     * Each of the above are bound to x/y/z respectively
     **/
    class LinearVector
    {
    public:
      /**
       * Primary constructor
       *
       * @param x the x coordinate of the new Linear
       * @param y the y coordinate of the new Linear
       * @param z the z coordinate of the new Linear; defaults to zero
       **/
      constexpr LinearVector (double x, double y, double z = 0.0);

      /**
       * Default constructor. Initializes an invalid Linear (INVAL_COORD).
       **/
      constexpr LinearVector ();

      /**
       * Constructor from C array
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit LinearVector (const double array[]);

      /**
       * Constructor from C array
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit LinearVector (const float array[]);

      /**
       * Constructor from MADARA DoubleVector
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      explicit LinearVector (
        const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      explicit LinearVector (
        const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Tests if this Linear is valid
       *
       * @return true if no values in this Linear are INVAL_COORD
       **/
      constexpr bool is_set () const;

      /**
       * Tests if all values in this Linear are the same
       * as the ones in the rhs.
       *
       * @param rhs the other Linear to check against
       **/
      constexpr bool operator== (const LinearVector &rhs) const;

      /**
       * Tests if all values in this Linear are zero
       *
       * @return true of all values are zero
       **/
      constexpr bool is_zero () const;

      /**
       * Returns the name of this coordinate type
       *
       * @return "Linear"
       **/
      static std::string name ();

      /**
       * Getter for x
       *
       * @return x value
       **/
      constexpr double x () const;

      /**
       * Getter for y
       *
       * @return y value
       **/
      constexpr double y () const;

      /**
       * Getter for z
       *
       * @return z value
       **/
      constexpr double z () const;

      /**
       * Setter for x
       *
       * @param new_x the new x value
       * @return new x value
       **/
      double x (double new_x);

      /**
       * Setter for y
       *
       * @param new_y the new y value
       * @return new y value
       **/
      double y (double new_y);

      /**
       * Setter for z
       *
       * @param new_z the new z value
       * @return new z value
       **/
      double z (double new_z);

      /**
       * Getter for longitude, a synonym for x
       *
       * @return longitude value
       **/
      constexpr double lng () const;
      constexpr double lon () const;
      constexpr double longitude () const;

      /**
       * Getter for latitude, a synonym for y
       *
       * @return latitude value
       **/
      constexpr double lat () const;
      constexpr double latitude () const;

      /**
       * Getter for altitude, a synonym for z
       *
       * @return altitude value
       **/
      constexpr double alt () const;
      constexpr double altitude () const;

      /**
       * Setter for longitude, a synonym for x
       *
       * @param new_lng new longitude value
       * @return new longitude value
       **/
      double lng (double new_lng);
      double lon (double new_lng);
      double longitude (double new_lng);

      /**
       * Setter for latitude, a synonym for y
       *
       * @param new_lat new latitude value
       * @return new latitude value
       **/
      double lat (double new_lat);
      double latitude (double new_lat);

      /**
       * Setter for altitude, a synonym for z
       *
       * @param new_alt new altitude value
       * @return new altitude value
       **/
      double alt (double new_alt);
      double altitude (double new_alt);

      /**
       * Getter for rho, a synonym for x for Cylindrical systems
       *
       * @return rho value
       **/
      constexpr double rho () const;

      /**
       * Getter for theta, a synonym for x for Spherical systems
       *
       * @return theta value
       **/
      constexpr double theta () const;

      /**
       * Getter for phi, a synonym for y for Cylindrical and Spherical systems
       *
       * @return phi value
       **/
      constexpr double phi () const;

      /**
       * Getter for r, a synonym for z for Cylindrical and Spherical systems
       *
       * @return r value
       **/
      constexpr double r () const;

      /**
       * Setter for rho, a synonym for x for Cylindrical systems
       *
       * @param new_rho new rho value
       * @return new rho value
       **/
      double rho (double new_rho);

      /**
       * Setter for theta, a synonym for x for Spherical systems
       *
       * @param new_theta new theta value
       * @return new theta value
       **/
      double theta (double new_theta);

      /**
       * Setter for phi, a synonym for y for Cylindrical and Spherical systems
       *
       * @param new_phi new phi value
       * @return new phi value
       **/
      double phi (double new_phi);

      /**
       * Setter for r, a synonym for z for Cylindrical and Spherical systems
       *
       * @param new_r new new value
       * @return new r value
       **/
      double r (double new_r);

#ifdef GAMS_UTM

      /**
       * Gets the UTM northing for this position.
       * Will return arbitrary value if this position is not bound to a UTMFrame
       *
       * Defined in UTMFrame.inl
       **/
      constexpr double northing () const;

      /**
       * Gets the UTM easting for this position.
       * Will return arbitrary value if this position is not bound to a UTMFrame
       *
       * Defined in UTMFrame.inl
       **/
      constexpr double easting () const;

      /**
       * Gets the UTM zone (longitudinal) for this position: 0 if USP (near
       * poles) otherwise in range [1, 60]
       * Will return arbitrary value if this position is not bound to a UTMFrame
       *
       * Defined in UTMFrame.inl
       **/
      constexpr int zone () const;

      /**
       * Gets the UTM hemisphere for this position. True if northern.
       * Will return arbitrary value if this position is not bound to a UTMFrame
       *
       * Defined in UTMFrame.inl
       **/
      constexpr bool hemi () const;

      /**
       * Set UTM northing and hemisphere (true is northern) simultaneously
       *
       * Defined in UTMFrame.inl
       *
       * @param n the new northing
       * @param h the new hemisphere
       **/
      void northing (double n, bool h);

      /**
       * Set UTM northing only; do not change hemisphere
       *
       * Defined in UTMFrame.inl
       *
       * @param n the new northing
       **/
      void northing (double n);

      /**
       * Set UTM hemisphere (true is northern) only; do not change northing
       *
       * Defined in UTMFrame.inl
       *
       * @param h the new hemisphere
       **/
      void hemi (bool h);

      /**
       * Set UTM easting and zone simultaneously
       *
       * Defined in UTMFrame.inl
       *
       * @param e the new easting
       * @param z the new zone; 0 for USP (polar regions), otherwise [1,60]
       **/
      void easting (double e, int z);

      /**
       * Set UTM easting only; do not change zone
       *
       * Defined in UTMFrame.inl
       *
       * @param e the new easting
       **/
      void easting (double e);

      /**
       * Set UTM zone only; do not change easting
       *
       * Defined in UTMFrame.inl
       *
       * @param z the new zone; 0 for USP (polar regions), otherwise [1,60]
       **/
      void zone (int z);

      /**
       * Get the NATO band code for the latitude of this UTM position
       *
       * Defined in UTMFrame.inl
       *
       * @return an uppercase letter according to NATO standards
       **/
      char nato_band ();

#endif // GAMS_UTM

      /**
       * Number of elements in this tuple
       *
       * @return the integer 3
       **/
      constexpr static int size ();

      /**
       * Retrives i'th coordinate, 0-indexed, in order x, y, z
       *
       * @param i the index
       * @return the i'th value
       **/
      constexpr double get (int i) const;

      /**
       * Sets i'th coordinate, 0-indexed, in order x, y, z
       *
       * @param i the index to set
       * @param val the value to set to
       **/
      double set (int i, double val);

      typedef LinearVector BaseType;

      /**
       * Returns reference to this as the above BaseType; useful
       * for derived types
       *
       * @return reference to this as BaseType
       **/
      BaseType &as_vec ();

      /**
       * Returns const reference to this as the above BaseType; useful
       * for derived types
       *
       * @return const reference to this as BaseType
       **/
      constexpr const BaseType &as_vec () const;

      friend class Quaternion;

      friend class ReferenceFrame;

    private:
      double x_, y_, z_;
    };

    /**
     * Represents a Linear within a reference frame.
     * This position always has x, y, and z coordinates, but interpretation
     * of those coordinates can vary according to the reference frame.
     *
     * Provides accessor methods to support non-cartesian coordinate systems:
     *
     * lng/lat/alt for GPS-style systems
     * rho/phi/r for Cylindrical systems
     * theta/phi/r for Spherical systems
     * northing/easting/zone/hemi/alt for UTM/USP systems
     *
     * Each of the above are bound to x/y/z respectively.
     *
     * Note that no checking is done to ensure that the correct accessors are
     * used based on the actual ReferenceFrame type this Linear is bound
     * to. In particular, the UTM/USP accessors may produce strange values if
     * the Linear is not actually bound to a UTMFrame, but will not cause
     * exceptions.
     **/
    class GAMSExport Linear : public LinearVector, public Coordinate<Linear>
    {
    public:
      /**
       * Primary constructor, using default reference frame.
       *
       * @param x the x coordinate of the new Linear
       * @param y the y coordinate of the new Linear
       * @param z the z coordinate of the new Linear; defaults to zero
       **/
      Linear (double x, double y, double z = 0.0);

      /**
       * Primary constructor
       *
       * @param frame the reference frame to bind to. This object must not
       *    outlive this ReferenceFrame object.
       * @param x the x coordinate of the new Linear
       * @param y the y coordinate of the new Linear
       * @param z the z coordinate of the new Linear; defaults to zero
       **/
      constexpr Linear (const ReferenceFrame &frame,
                         double x, double y, double z = 0.0);

      /**
       * Constructor from C array, into default ReferenceFrame
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit Linear (const double array[]);

      /**
      * Copy constructor
      * @param rhs the source of the copy
      **/
      Linear (const Linear & rhs);

      /**
       * Constructor from C array, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit Linear (const ReferenceFrame &frame, const double array[]);

      /**
       * Constructor from C array, into default ReferenceFrame
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit Linear (const float array[]);

      /**
       * Constructor from C array, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param array the array to get values from (index 0, 1, 2 go to x, y, z)
       *              if array's size is less than 3, behavior is undefined
       **/
      explicit Linear (const ReferenceFrame &frame, const float array[]);

      /**
       * Constructor from MADARA DoubleVector, into default ReferenceFrame
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      explicit Linear (
              const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA DoubleVector, into specified ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      Linear (const ReferenceFrame &frame,
               const madara::knowledge::containers::DoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into default
       * ReferenceFrame
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      explicit Linear (
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Constructor from MADARA NativeDoubleVector, into specified
       * ReferenceFrame
       * @param frame the frame to belong to
       * @param vec the vector to get values from (index 0, 1, 2 go to x, y, z)
       **/
      Linear (const ReferenceFrame &frame,
         const madara::knowledge::containers::NativeDoubleVector &vec);

      /**
       * Default constructor. Initializes an invalid Linear (INVAL_COORD).
       **/
      Linear ();

      /**
       * Constructs an invalid Linear (INVAL_COORD), in the given frame
       *
       * @param frame the frame to belong to
       **/
      Linear (const ReferenceFrame &frame);

      /**
       * Copy constructor, but transform into the new frame as well.
       *
       * @param new_frame the new frame to transform to
       * @param orig    the origin position to use as a reference point
       **/
      Linear (const ReferenceFrame &new_frame, const Linear &orig);

      /**
      * Returns a string of the values x, y, z
      * @param delimiter      delimiter between values
      * @param unset_identifier  value to print if unset
      * @return  stringified version of the Linear
      **/
      std::string to_string (
        const std::string & delimiter = ", ",
        const std::string & unset_identifier = "<unset>") const;

      /**
      * Saves the position to a MADARA container
      * @param  container the container to save to
      **/
      void to_container (
        madara::knowledge::containers::NativeDoubleVector &container) const;

      /**
      * Imports the position from a MADARA container
      * @param  container the container to import from
      **/
      void from_container (
        const madara::knowledge::containers::NativeDoubleVector &container);


      using Coordinate<Linear>::operator==;
    };

    // helpful typedef for vector of positions
    typedef std::vector <Linear>    Linears;
  }
}

#include "Linear.inl"

#endif
