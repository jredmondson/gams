/**
 * Copyright(c) 2014 Carnegie Mellon University. All Rights Reserved.
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

#include "ReferenceFrame.h"

#ifndef  _GAMS_UTILITY_REGION_H_
#define  _GAMS_UTILITY_REGION_H_

#include <vector>
#include <string>

#include "gams/GamsExport.h"
#include "madara/knowledge/containers/StringVector.h"

#include "gams/pose/Position.h"
#include "gams/pose/GPSFrame.h"
#include "gams/pose/CartesianFrame.h"

#include "gams/utility/Containerize.h"

namespace gams
{
  namespace pose
  {
    /**
     * A helper class for region information
     **/
    class GAMS_EXPORT Region : public utility::Containerize
    {
    public:
      /**
       * Constructor
       * @param  init_vertices  the vertices of the region
       * @param  type           the type of region
       * @param  name           name of the region
       **/
      Region(const std::vector <Position> & init_vertices = 
        std::vector<Position>(), unsigned int type = 0, 
        const std::string& name = "");

      /**
       * Destructor
       **/
      virtual ~Region();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator=(const Region& rhs);

      /**
       * Equality operator. Only checks if the vertices are the same
       * @param rhs   Region to compare with
       */
      bool operator==(const Region& rhs) const;

      /**
       * Inequality operator. Calls operator== and inverses result
       * @param rhs   Region to compare with
       */
      bool operator!=(const Region& rhs) const;

      /**
       * Gets name of region
       * @return name of the region
       */
      std::string get_name() const;

      /**
       * Sets name of region
       * @param name   new name of the region
       */
      void set_name(const std::string& name);

      /**
       * Determines if GPSPosition is in region
       * @param   position   point to check if in region
       * @return  true if point is in region or on border, false otherwise
       **/
      bool contains(const Position & position) const;

      /**
       * Gets distance from any point in this region
       * @param   position     point to check
       * @return 0 if in region, otherwise distance from region
       **/
      double distance(const Position & position) const;

      /**
       * Gets bounding box
       * @return Region object corresponding to bounding box
       **/
      Region get_bounding_box() const;

      /**
       * Gets area of the region
       * @return area of this region
       **/
      double get_area() const;

      /**
       * Converts the position to a string
       * @param delimiter characters to insert between position components
       * @return string representation of this Region
       **/
      std::string to_string(const std::string & delimiter = ":") const;

      /// the vertices of the region
      std::vector <Position> vertices;

      /// bounding box
      double min_lat_, max_lat_;
      double min_lon_, max_lon_;
      double min_alt_, max_alt_;

    protected:
      /**
       * populate bounding box values
       **/
      void calculate_bounding_box();

      /// type for this region
      unsigned int type_;

    private:
      /**
       * Check if object is of correct type
       * @param kb        Knowledge Base with object
       * @param name      Name of object in the KB
       * @return true if name is a valid object in kb, false otherwise
       */
      virtual bool check_valid_type(madara::knowledge::KnowledgeBase& kb,
        const std::string& name) const;

      /**
       * Store object in knowledge base
       * @param kb        Knowledge Base to store object in
       * @param name      location of object in Knowlege Base
       **/
      virtual void to_container_impl(
        madara::knowledge::KnowledgeBase& kb, 
        const std::string& name);

      /**
       * Load object from knowledge base
       * @param kb        Knowledge Base with object
       * @param name      location of object in Knowlege Base
       **/
      virtual bool from_container_impl(
        madara::knowledge::KnowledgeBase& kb, 
        const std::string& name);
    }; // class Region
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_REGION_H_
