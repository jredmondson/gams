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
 * @file Prioritized_Region.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Prioritized region associates a priority with a region
 **/

#ifndef  _GAMS_UTILITY_PRIORITIZED_REGION_H_
#define  _GAMS_UTILITY_PRIORITIZED_REGION_H_

#include <string>
#include <vector>

#include "gams/utility/Region.h"
#include "gams/utility/GPS_Position.h"

namespace gams
{
  namespace utility
  {
    class GAMS_Export Prioritized_Region : public Region
    {
    public:
      /**
       * Constructor
       * @param init_points vector of points representing boundary polygon of region
       * @param new_priority  associated priority
       **/
      Prioritized_Region (const std::vector <GPS_Position>& init_points =
        std::vector<GPS_Position> (), const unsigned int new_priority = 1);

      /**
       * Constructor
       * @param region    associated region
       * @param new_priority  associated priority
       **/
      Prioritized_Region (const Region& region, const unsigned int new_priority = 1);
      
    /**
     * Initialize prioritized region from knowledge base
     * @param knowledge   knowledge base to draw from
     * @param prefix      prefix for the region (e.g. "region.0")
     **/
      void init (Madara::Knowledge_Engine::Knowledge_Base & knowledge,
        const std::string & prefix);

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const Prioritized_Region& rhs);

      /// priority
      Madara::Knowledge_Record::Integer priority;
    }; // class Prioritized_Region

    /**
     * Create prioritized region from knowledge base information
     * @param knowledge   knowledge base to draw from
     * @param prefix      prefix for the region (e.g. "region.0")
     * @return Prioritized_Region object created from knowledge base
     **/
    Prioritized_Region parse_prioritized_region (
      Madara::Knowledge_Engine::Knowledge_Base & knowledge,
      const std::string & prefix);
  } // namespace utility
} // namespace gams

#endif // _GAMS_UTILITY_PRIORITIZED_REGION_H_
