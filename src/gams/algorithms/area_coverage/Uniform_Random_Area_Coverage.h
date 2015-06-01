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
 * @file Uniform_Random_Area_Coverage.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the random area coverage class
 **/

#ifndef   _GAMS_ALGORITHMS_AREA_COVERAGE_UNIFORM_RANDOM_AREA_COVERAGE_H_
#define   _GAMS_ALGORITHMS_AREA_COVERAGE_UNIFORM_RANDOM_AREA_COVERAGE_H_

#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/algorithms/area_coverage/Base_Area_Coverage.h"

#include "gams/platforms/Base_Platform.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/utility/Region.h"

namespace gams
{
  namespace algorithms
  {
    namespace area_coverage
    {
      class GAMS_Export Uniform_Random_Area_Coverage : public Base_Area_Coverage
      {
      public:
        /**
         * Constructor
         * @param search_area_id  identifier of region to cover
         * @param knowledge       the context containing variables and values
         * @param platform        the underlying platform the algorithm will use
         * @param sensors         map of sensor names to sensor information
         * @param self            self-referencing variables
         **/
        Uniform_Random_Area_Coverage (
          const Madara::Knowledge_Record& search_area_id,
          Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
          platforms::Base_Platform * platform = 0, variables::Sensors * sensors = 0,
          variables::Self * self = 0);
  
        /**
         * Destructor
         **/
        ~Uniform_Random_Area_Coverage ();
  
        /**
         * Assignment operator
         * @param  rhs   values to copy
         **/
        void operator= (const Uniform_Random_Area_Coverage & rhs);
        
      protected:
        /**
         * Generate new next position
         */
        virtual void generate_new_position ();

        /// region to cover
        utility::Region region_;
      };

      /**
       * A factory class for creating uniform random area
       * coverage algorithms
       **/
      class GAMS_Export Uniform_Random_Area_Coverage_Factory
        : public Algorithm_Factory
      {
      public:

        /**
         * Creates a uniform random area coverage algorithm
         * @param   args      args[0] = region id
         * @param   platform  the platform. This will be set by the
         *                    controller in init_vars.
         * @param   sensors   the sensor info. This will be set by the
         *                    controller in init_vars.
         * @param   self      self-referencing variables. This will be
         *                    set by the controller in init_vars
         * @param   devices   the list of devices, which is dictated by
         *                    init_vars when a number of processes is set. This
         *                    will be set by the controller in init_vars
         **/
        virtual Base_Algorithm * create (
          const Madara::Knowledge_Vector & args,
          Madara::Knowledge_Engine::Knowledge_Base * knowledge,
          platforms::Base_Platform * platform,
          variables::Sensors * sensors,
          variables::Self * self,
          variables::Devices * devices);
      };
    } // namespace area_coverage
  } // namespace algorithms
} // namespace gams

#endif // _GAMS_ALGORITHMS_AREA_COVERAGE_UNIFORM_RANDOM_AREA_COVERAGE_H_
