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
 * @file PrioritizedRegion.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized region associates a priority with a region
 **/

#include "ReferenceFrame.h"

#ifndef  _GAMS_UTILITY_PRIORITIZED_REGION_H_
#define  _GAMS_UTILITY_PRIORITIZED_REGION_H_

#include <string>
#include <vector>

#include "gams/pose/Region.h"
#include "gams/pose/Position.h"

namespace gams
{
  namespace pose
  {
    /**
    * A helper class for prioritized region information
    **/
    class GAMSExport PrioritizedRegion : public Region
    {
    public:
      /**
       * Constructor
       * @param init_points   vector of points representing boundary polygon of region
       * @param new_priority  associated priority
       * @param name          name of the region
       **/
      PrioritizedRegion (const std::vector <Position>& init_points =
        std::vector<Position> (), const unsigned int new_priority = 1, const std::string& name = "");

      /**
       * Constructor
       * @param region        associated region
       * @param new_priority  associated priority
       * @param name          name of the region
       **/
      PrioritizedRegion (const Region& region, const unsigned int new_priority = 1, const std::string& name = "");

      /**
       * Destructor
       **/
      virtual ~PrioritizedRegion ();

      /**
       * Equality operator. Uses Region::operator== and checks if priority are equal
       * @param rhs   PrioritizedRegion to compare to
       * @return true if same vertices and same priority, false otherwise
       **/
      bool operator== (const PrioritizedRegion& rhs) const;

      /**
       * Inequality operator. Uses operator== and inverses result
       * @param rhs   PrioritizedRegion to compare to
       * @return false if same vertices and same priority, true otherwise
       **/
      bool operator!= (const PrioritizedRegion& rhs) const;

      /**
       * Helper function for converting the position to a string
       * @param delimiter characters to insert between position components
       * @return string representation of this PrioritizedRegion
       **/
      std::string to_string (const std::string & delimiter = ":") const;

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const PrioritizedRegion& rhs);

      /// priority
      madara::knowledge::KnowledgeRecord::Integer priority;

    private:
      /**
       * Check if object is of correct type
       * @param kb        Knowledge Base with object
       * @param name      Prefix of object in the KB
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
    }; // class PrioritizedRegion
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_PRIORITIZED_REGION_H_
