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
 * @file Location.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the Location and LocationVector classes
 **/

#ifndef _GAMS_UTILITY_LOCATION_H_
#define _GAMS_UTILITY_LOCATION_H_

#include <gams/pose/Position.h>
#include <gams/utility/Coordinate.h>

namespace gams
{
  namespace utility
  {
    /**
     * Container for Location information, not bound to a frame.
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
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    typedef gams::pose::LinearVector LocationVector;

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
     * northing/easting/zone/hemi/alt for UTM/USP systems
     *
     * Each of the above are bound to x/y/z respectively.
     *
     * Note that no checking is done to ensure that the correct accessors are
     * used based on the actual ReferenceFrame type this Location is bound
     * to. In particular, the UTM/USP accessors may produce strange values if
     * the Location is not actually bound to a UTMFrame, but will not cause
     * exceptions.
     *
     * Deprecated backwards compatibility aliases. Will be removed in v2
     **/
    typedef gams::pose::Position Location;
  }
}

#endif
