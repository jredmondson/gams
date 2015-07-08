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
 * @file Executive.h
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * Declaration of Executive class
 **/

#ifndef _GAMS_ALGORITHMS_EXECUTIVE_H_
#define _GAMS_ALGORITHMS_EXECUTIVE_H_

#include "gams/variables/Sensor.h"
#include "gams/platforms/Base_Platform.h"
#include "gams/algorithms/Formation_Flying.h"
#include "gams/variables/Algorithm_Status.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/Base_Algorithm.h"
#include "gams/algorithms/Algorithm_Factory.h"
#include "gams/algorithms/Controller_Algorithm_Factory.h"

namespace gams
{
  namespace algorithms
  {
    class GAMS_Export Executive : public Base_Algorithm
    {
    public:
      /**
       * Constructor
       * @param  args         plan information
       * @param  knowledge    the context containing variables and values
       * @param  platform     the underlying platform the algorithm will use
       * @param  sensors      map of sensor names to sensor information
       * @param  self         self-referencing variables
       * @param  devices      variables referencing devices
       **/
      Executive (
        const Madara::Knowledge_Vector & args,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge = 0,
        platforms::Base_Platform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Devices * devices = 0);

      /**
       * Destructor
       **/
      ~Executive ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (Executive & rhs);
      
      /**
       * Analyzes environment, platform, or other information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void);
      
      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int execute (void);

      /**
       * Plans the next execution of the algorithm
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int plan (void);
      
    protected:
      /**
       * Algorithm_Init data keeps track of algorithm information
       */
      struct Algorithm_Init
      {
        std::string algorithm;
        Madara::Knowledge_Vector args;

        Algorithm_Init ();
        Algorithm_Init (const std::string& a,
          const Madara::Knowledge_Vector& v);
      };

      /// algorithm actually being run
      Base_Algorithm* algo_;

      /// index into plan
      size_t plan_index_;

      /// plan vector
      std::vector<Algorithm_Init> plan_;

      /// Subalgorithm constructor
      Controller_Algorithm_Factory algo_factory_;
    };
    
    /**
     * A factory class for creating Executive algorithms
     **/
    class GAMS_Export Executive_Factory : public Algorithm_Factory
    {
    public:
      /**
       * Creates an Executive Algorithm.
       * @param   args      n sets of args
       *                    args[i] = algorithm to run
       *                    args[i+1] = madara variable with arguments for the algorithm
       * @param   knowledge the knowledge base to use
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
  }
}

#endif // _GAMS_ALGORITHMS_EXECUTIVE_H_
