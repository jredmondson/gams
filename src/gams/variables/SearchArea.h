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
 * @file SearchArea.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the search area-prefixed MADARA variables
 **/

#ifndef   _GAMS_VARIABLES_SEARCH_AREA_H_
#define   _GAMS_VARIABLES_SEARCH_AREA_H_

#include <vector>

#include "Region.h"
#include "madara/knowledge/KnowledgeBase.h"

namespace gams
{
  namespace variables
  {
    /**
    * A container for search area information
    **/
    class GAMSExport SearchArea
    {
    public:
      /**
       * Constructor
       **/
      SearchArea ();

      /**
       * Destructor
       **/
      ~SearchArea ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const SearchArea & rhs);

      /**
       * Initializes variable containers
       * @param   knowledge  the knowledge base that houses the variables
       * @param   area_name  name of the area
       **/
      void init_vars (madara::knowledge::KnowledgeBase & knowledge,
      const std::string & area_name = "0");
      
      /**
       * Initializes variable containers
       * @param   knowledge  the variable context
       * @param   area_name  name of the area
       **/
      void init_vars (madara::knowledge::Variables & knowledge,
      const std::string & area_name = "0");

      /// region that the named search area points to
      //Region region;

      /// name of the search area
      std::string name;
    };
    
    /**
      * Initializes search area containers
      * @param   variables  the variables to initialize
      * @param   knowledge  the knowledge base that houses the variables
      * @param   area_name  name of the area
      **/
    GAMSExport void init_vars (SearchArea & variables,
      madara::knowledge::KnowledgeBase & knowledge,
      const std::string & area_name = "0");
  }
}

#endif // _GAMS_VARIABLES_SEARCH_AREA_H_
