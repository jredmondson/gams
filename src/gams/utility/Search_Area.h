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
 * @file Search_Area.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Search Area is a collection of regions, possibly with priority
 **/

#ifndef  _GAMS_UTILITY_SEARCH_AREA_H_
#define  _GAMS_UTILITY_SEARCH_AREA_H_

#include <vector>
#include <string>

#include "gams/utility/Prioritized_Region.h"

namespace gams
{
  namespace utility
  {
    class GAMS_Export Search_Area : public Containerize
    {
    public:
      /**
       * Default constructor
       **/
      Search_Area ();

      /**
       * Constructor
       * @param  region   the initial region of the search area
       **/
      Search_Area (const Prioritized_Region& region);

      /**
       * Constructor
       * @param regions   regions in this search area
       */
      Search_Area (const std::vector<Prioritized_Region>& regions);

      /**
       * Destructor
       **/
      ~Search_Area ();

      /**
       * Equality operator
       * @param rhs   Search_Area to compare against
       * @return true if Search_Areas have same Prioritized_Region, false otherwise
       **/
      bool operator== (const Search_Area& rhs) const;

      /**
       * Inequality operator, uses Equality operator and inverses it
       * @param rhs   Search_Area to compare against
       * @return false if Search_Areas have same Prioritized_Region, true otherwise
       **/
      bool operator!= (const Search_Area& rhs) const;

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Search_Area & rhs);

      /**
       * Get Search_Area name
       * @return name of the search area
       **/
      std::string get_name () const;

      /**
       * Set Search_Area name
       * @param n     new name for Search_Area
       **/
      void set_name (const std::string& n);

      /**
       * Add prioritized region to search area
       * @param r   prioritized region to add
       **/
      void add_prioritized_region (const Prioritized_Region& r);

      /**
       * Find the convex hull
       * @return Convex hull of the regions
       **/
      Region get_convex_hull () const;

      /**
       * Get region data
       * @return const reference to regions
       **/
      const std::vector<Prioritized_Region>& get_regions () const;

      /**
       * Get priority of a gps position
       * @param pos   position to get priority of
       * @return priority of position
       */
      Madara::Knowledge_Record::Integer get_priority (const GPS_Position& pos) const;
      
      /**
       * Determine if GPS_Position is in region
       * @param   p   point to check if in region
       * @return  true if point is in the search area, false otherwise
       **/
      bool contains (const GPS_Position& p) const;
      
      /**
       * Create string representation of Search_Area
       * @return string representation of this object
       **/
      std::string to_string () const;

      /**
       * Helper function for copying values to a MADARA knowledge base
       * @param kb        knowledge base to store region information
       **/
      void to_container (Madara::Knowledge_Engine::Knowledge_Base& kb);

      /**
       * Helper function for copying values to a MADARA knowledge base
       * @param kb        knowledge base to store region information
       * @param name      name of the region
       **/
      void to_container (Madara::Knowledge_Engine::Knowledge_Base& kb,
        const std::string& name);

      /**
       * Helper function for copying values from a MADARA knowledge base, uses name_
       * @param kb        knowledge base with region information
       * @return true if valid object set, false otherwise
       **/
      bool from_container (Madara::Knowledge_Engine::Knowledge_Base& kb);
      
      /**
       * Helper function for copying values from a MADARA knowledge base
       * @param kb        knowledge base with region information
       * @param name      name of the region to get
       * @return true if valid object set, false otherwise
       **/
      bool from_container (Madara::Knowledge_Engine::Knowledge_Base& kb,
        const std::string& name);

      /// bounding box
      double min_lat_, max_lat_;
      double min_lon_, max_lon_;
      double min_alt_, max_alt_;

    protected:
      /**
       * populate bounding box values
       **/
      void calculate_bounding_box ();

      /**
       * Helper function for convex hull calculations
       * @param gp1  start point
       * @param gp2  pivot point
       * @param gp3  end point
       * @return cross product of the points
       **/
      double cross (const GPS_Position& gp1, const GPS_Position& gp2, 
        const GPS_Position& gp3) const;

      /// collection of prioritized regions
      std::vector<Prioritized_Region> regions_;

      /// name of this search area
      std::string name_;

    private:
      /**
       * Check if object is of correct type
       * @param kb        Knowledge Base with object
       * @param prefix    Prefix of object in the KB
       */
      virtual bool check_valid_type (Madara::Knowledge_Engine::Knowledge_Base& kb,
        const std::string& name) const;
    }; // class Search_Area
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_SEARCH_AREA_H_
