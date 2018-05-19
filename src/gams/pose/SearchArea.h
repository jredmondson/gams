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
 * @file SearchArea.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Search Area is a collection of regions, possibly with priority
 **/

#include "ReferenceFrame.h"

#ifndef  _GAMS_UTILITY_SEARCH_AREA_H_
#define  _GAMS_UTILITY_SEARCH_AREA_H_

#include <vector>
#include <string>

#include "gams/pose/PrioritizedRegion.h"

namespace gams
{
  namespace pose
  {
    /**
    * A utility class for search areas
    **/
    class GAMSExport SearchArea : public utility::Containerize
    {
    public:
      /**
       * Default constructor
       **/
      SearchArea ();

      /**
       * Constructor
       * @param region  the initial region of the search area
       * @param name    name for this search area
       **/
      SearchArea (const PrioritizedRegion& region, 
        const std::string& name = "");

      /**
       * Constructor
       * @param regions regions in this search area
       * @param name    name for this search area
       */
      SearchArea (const std::vector<PrioritizedRegion>& regions,
        const std::string& name = "");

      /**
       * Destructor
       **/
      ~SearchArea ();

      /**
       * Equality operator
       * @param rhs   SearchArea to compare against
       * @return true if SearchAreas have same PrioritizedRegion, false otherwise
       **/
      bool operator== (const SearchArea& rhs) const;

      /**
       * Inequality operator, uses Equality operator and inverses it
       * @param rhs   SearchArea to compare against
       * @return false if SearchAreas have same PrioritizedRegion, true otherwise
       **/
      bool operator!= (const SearchArea& rhs) const;

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const SearchArea & rhs);

      /**
       * Add prioritized region to search area
       * @param r   prioritized region to add
       **/
      void add_prioritized_region (const PrioritizedRegion& r);

      /**
       * Find the convex hull
       * @return Convex hull of the regions
       **/
      Region get_convex_hull () const;

      /**
       * Get region data
       * @return const reference to regions
       **/
      const std::vector<PrioritizedRegion>& get_regions () const;

      /**
       * Get priority of a gps position
       * @param pos   position to get priority of
       * @return priority of position
       */
      madara::knowledge::KnowledgeRecord::Integer get_priority (const Position& pos) const;
      
      /**
       * Determine if Position is in region
       * @param   p   point to check if in region
       * @return  true if point is in the search area, false otherwise
       **/
      bool contains (const Position& p) const;
      
      /**
       * Create string representation of SearchArea
       * @return string representation of this object
       **/
      std::string to_string () const;

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
      double cross (const Position& gp1, const Position& gp2, 
        const Position& gp3) const;

      /// collection of prioritized regions
      std::vector<PrioritizedRegion> regions_;

    private:
      /**
       * Check if object is of correct type
       * @param kb        Knowledge Base with object
       * @param name      Name of object in the KB
       */
      virtual bool check_valid_type (madara::knowledge::KnowledgeBase& kb,
        const std::string& name) const;

      /**
       * Store object in knowledge base
       * @param kb        Knowledge Base to store object in
       * @param name      location of object in Knowlege Base
       **/
      virtual void to_container_impl (
        madara::knowledge::KnowledgeBase& kb, 
        const std::string& name);

      /**
       * Load object from knowledge base
       * @param kb        Knowledge Base with object
       * @param name      location of object in Knowlege Base
       **/
      virtual bool from_container_impl (
        madara::knowledge::KnowledgeBase& kb, 
        const std::string& name);
    }; // class SearchArea
  } // namespace utility
} // namespace gams

#include "SearchArea.inl"

#endif // _GAMS_UTILITY_SEARCH_AREA_H_
