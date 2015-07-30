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
 * @file Region.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a utility class for working with position
 **/

#ifndef  _GAMS_UTILITY_REGION_H_
#define  _GAMS_UTILITY_REGION_H_

#include <vector>
#include <string>

#include "gams/GAMS_Export.h"
#include "madara/knowledge_engine/containers/String_Vector.h"
#include "gams/utility/Position.h"
#include "gams/utility/GPS_Position.h"

namespace gams
{
  namespace utility
  {
    class GAMS_Export Region
    {
    public:
      /**
       * Constructor
       * @param  init_vertices  the vertices of the region
       **/
      Region (const std::vector <GPS_Position> & init_vertices = 
        std::vector<GPS_Position> (), unsigned int t = 0, 
        const std::string& name = "");

      /**
       * Destructor
       **/
      ~Region ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Region& rhs);

      /**
       * Equality operator. Only checks if the vertices are the same
       * @param rhs   Region to compare with
       */
      bool operator== (const Region& rhs) const;

      /**
       * Inequality operator. Calls operator== and inverses result
       * @param rhs   Region to compare with
       */
      bool operator!= (const Region& rhs) const;

      /**
       * Get name of region
       * @return name of the region
       */
      std::string get_name () const;

      /**
       * Set name of region
       * @param n   new name of the region
       */
      void set_name (const std::string& n);
      
      /**
       * Determine if GPS_Position is in region
       * @param   p   point to check if in region
       * @return  true if point is in region or on border, false otherwise
       **/
      bool contains (const GPS_Position & p) const;

      /**
       * Determine if GPS_Position is in region
       * @param   p     point to check if in region
       * @param   ref   gps reference for point p
       * @return  true if point is in region or on border, false otherwise
       **/
      bool contains (const Position & p, const GPS_Position& ref) const;

      /**
       * Get distance from any point in this region
       * @param   p     point to check
       * @return 0 if in region, otherwise distance from region
       **/
      double distance (const GPS_Position& p) const;

      /**
       * Get bounding box
       * @return Region object corresponding to bounding box
       **/
      Region get_bounding_box () const;

      /**
       * Get area of the region
       * @return area of this region
       **/
      double get_area () const;

      /**
       * Helper function for converting the position to a string
       * @param delimiter characters to insert between position components
       * @return string representation of this Region
       **/
      std::string to_string (const std::string & delimiter = ":") const;

      /**
       * Helper function for copying values to a MADARA knowledge base
       * @param kb        knowledge base to store region information
       **/
      void to_container (Madara::Knowledge_Engine::Knowledge_Base& kb) const;

      /**
       * Helper function for copying values to a MADARA knowledge base
       * @param kb        knowledge base to store region information
       * @param name      name of the region
       **/
      void to_container (Madara::Knowledge_Engine::Knowledge_Base& kb,
        const std::string& name) const;

      /**
       * Helper function for copying values from a MADARA knowledge base
       * @param kb        knowledge base with region information
       **/
      void from_container (Madara::Knowledge_Engine::Knowledge_Base& kb);
      
      /**
       * Helper function for copying values from a MADARA knowledge base
       * @param kb        knowledge base with region information
       * @param name      name of the region to get
       **/
      void from_container (Madara::Knowledge_Engine::Knowledge_Base& kb,
        const std::string& name);

      /// the vertices of the region
      std::vector <GPS_Position> vertices;

      /// bounding box
      double min_lat_, max_lat_;
      double min_lon_, max_lon_;
      double min_alt_, max_alt_;

    protected:
      /// name of the region
      std::string name_;
      
      /**
       * populate bounding box values
       **/
      void calculate_bounding_box ();

      /// type for this region
      unsigned int type_;
    }; // class Region
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_REGION_H_
