/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
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
 * 3. The names Carnegie Mellon University, "SEI and/or Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN AS-IS BASIS. CARNEGIE MELLON
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
 * @file GPS_Position.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains a utility class for working with gps coordinates
 **/

#ifndef _GAMS_UTILITY_GPS_POSITION_H_
#define _GAMS_UTILITY_GPS_POSITION_H_

#include "gams/GAMS_Export.h"
#include "madara/knowledge_engine/containers/Double_Vector.h"
#include "madara/knowledge_engine/containers/Native_Double_Vector.h"

#include "Position.h"

namespace gams
{
  namespace utility
  {
    class GAMS_Export GPS_Position : public Position
    {
    public:
      /**
       * Constructor
       * @param  init_lat   the initial latitude
       * @param  init_lon   the initial longitude
       * @param  init_alt   the initial altitude
       **/
      GPS_Position (
        double init_lat = 0.0, double init_lon = 0.0, double init_alt = 0.0);

      /**
       * Constructor
       **/
      GPS_Position (const GPS_Position & position);
      
      /**
       * Constructor
       **/
      GPS_Position (const Position & position);

      /**
       * Destructor
       **/
      virtual ~GPS_Position ();

      /**
       * Ordering operator for sets
       **/
      bool operator< (const GPS_Position& rhs) const;

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const GPS_Position & rhs);

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if all members are equal in both objects, false otherwise
       **/
      bool operator== (const GPS_Position & rhs) const;
      
      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if all members are equal in both objects, false otherwise
       **/
      bool operator== (const
        Madara::Knowledge_Engine::Containers::Double_Array & rhs) const;

      /**
       * Equality operator
       * @param  rhs   value to compare
       * @return true if all members are equal in both objects, false otherwise
       **/
      bool operator== (const
        Madara::Knowledge_Engine::Containers::Native_Double_Array & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if any members are not equal in both objects, false otherwise
       **/
      bool operator!= (const GPS_Position & rhs) const;
      
      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if any members are not equal in both objects, false otherwise
       **/
      bool operator!= (const
        Madara::Knowledge_Engine::Containers::Double_Array & rhs) const;

      /**
       * Inequality operator
       * @param  rhs   value to compare
       * @return true if any members are not equal in both objects, false otherwise
       **/
      bool operator!= (const
        Madara::Knowledge_Engine::Containers::Native_Double_Array & rhs) const;

      /**
       * Approximate equality
       * @param  rhs      value to compare
       * @param  epsilon  approximation value in meters
       * @return true if position is within distance epsilon of *this
       **/
      virtual bool approximately_equal(
        const GPS_Position & rhs, const double & epsilon) const;

      /**
       * Get spherical direction to position
       * @param rhs     other position
       * @param phi     direction to rhs
       **/
      virtual void direction_to (const GPS_Position& rhs, double& phi) const;

      /**
       * Get distance between two positions
       * @param  rhs      second position
       * @return euclidean distance between the two points in meters
       **/
      virtual double distance_to (const GPS_Position & rhs) const;

      /**
       * Helper function for converting the position to a string
       * @param delimiter characters to insert between position components
       * @param precision precision of doubles when printing coordinates
       **/
      virtual std::string to_string (const std::string & delimiter = ",",
        const unsigned int precision = 8) const;

      /**
       * Convert to position using reference location
       * @param ref   Reference location
       * @return Position object relative to ref
       **/
      virtual Position to_position (const GPS_Position& ref) const;
      
      /**
       * Convert source to a GPS_Position
       * @param source   source position to convert
       * @param ref      origin GPS_Position
       * @return GPS_Position corresponding to source
       **/
      static GPS_Position to_gps_position (const Position & source,
        const GPS_Position & ref);

      /**
       * Helper function for creating a GPS_Position from a string
       * @param delimiter characters to insert between position components
       **/
      static GPS_Position from_string (const std::string & delimiter = ",");

      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container (
        Madara::Knowledge_Engine::Containers::Double_Array & target) const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container (
        Madara::Knowledge_Engine::Containers::Double_Array & source);
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param target     target container to copy values to
       **/
      virtual void to_container (
        Madara::Knowledge_Engine::Containers::Native_Double_Array & target)
        const;
      
      /**
       * Helper function for copying values to a MADARA double array
       * @param source     source container to copy values from
       **/
      virtual void from_container (
        Madara::Knowledge_Engine::Containers::Native_Double_Array & source);

      /**
       * Returns the latitude of the GPS coordinate
       **/
      inline double latitude () const { return x; }
      
      /**
       * Returns the longitude of the GPS coordinate
       **/
      inline double longitude () const { return y; }
      
      /**
       * Returns the altitude of the GPS coordinate
       **/
      inline double altitude () const { return z; }

      /**
       * Sets the latitude to the input
       **/
      inline void latitude (double input) { x = input; }
      
      /**
       * Sets the longitude to the input
       **/
      inline void longitude (double input) { y = input; }
      
      /**
       * Sets the altitude to the input
       **/
      inline void altitude (double input) { z = input; }

    }; // class GPS_Position
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_GPS_POSITION_H_
