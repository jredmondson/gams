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
 * @file JavaAlgorithm.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains the definition of the java algorithm abstraction
 **/

#ifndef   _GAMS_ALGORITHM_JAVA_H_
#define   _GAMS_ALGORITHM_JAVA_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/utility/GPSPosition.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "gams/algorithms/AlgorithmFactory.h"

#ifdef _GAMS_JAVA_
#include <jni.h>
#include "gams_jni.h"
#endif

namespace gams
{
  namespace algorithms
  {
    /**
    * A facade for Java algorithms
    **/
    class GAMS_EXPORT JavaAlgorithm : public BaseAlgorithm
    {
    public:
      /**
       * Constructor
       * @param  obj        the Java object to call methods on
       * @param  knowledge  knowledge base
       * @param  platform   the platform to use
       * @param  sensors    map of sensor names to sensor information
       * @param  self       agent variables that describe self state
       * @param  agents    list of participating agents
       **/
      JavaAlgorithm (
        jobject obj,
        madara::knowledge::KnowledgeBase * knowledge = 0,
        platforms::BasePlatform * platform = 0,
        variables::Sensors * sensors = 0,
        variables::Self * self = 0,
        variables::Agents * agents = 0);

      /**
       * Destructor
       **/
      ~JavaAlgorithm ();

      /**
       * Assignment operator
       * @param  rhs   values to copy
       **/
      void operator= (const JavaAlgorithm & rhs);
      
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
      
      /**
       * Gets the unique identifier of the algorithm. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. rac, follow_leader, etc.)
       **/
      virtual std::string get_id () const;

      /**
       * Gets the name of the algorithm
       **/
      virtual std::string get_name () const;
      
      /**
       * Returns the Java instance that derives from
       * BaseAlgorithm.
       **/
      jobject get_java_instance (void);

    protected:
      /// the Java object with callable methods
      jobject obj_;

      /// the class of the Java object obj_
      jclass class_;
    };


    /**
    * A factory class for creating Java Algorithms
    **/
    class GAMS_EXPORT JavaAlgorithmFactory : public AlgorithmFactory
    {
    public:

      /**
       * Constructor
       * @param obj  the Java object that implements AlgorithmFactory
       **/
      JavaAlgorithmFactory (jobject obj);

      /**
       * Destructor
       **/
      virtual ~JavaAlgorithmFactory ();

      /**
      * Creates a Java Algorithm.
      * @param   args    first arg is where to store the executions tracker in
      *                  the knowledge base. Default is ".executions" when no
      *                  args are provided.
      * @param   knowledge the knowledge base to use
      * @param   platform  the platform. This will be set by the
      *                    controller in init_vars.
      * @param   sensors   the sensor info. This will be set by the
      *                    controller in init_vars.
      * @param   self      self-referencing variables. This will be
      *                    set by the controller in init_vars
      * @param   agents   the list of agents, which is dictated by
      *                    init_vars when a number of processes is set. This
      *                    will be set by the controller in init_vars
      **/
      virtual BaseAlgorithm * create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        platforms::BasePlatform * platform,
        variables::Sensors * sensors,
        variables::Self * self,
        variables::Agents * agents);

      /**
      * Returns the Java instance that implements from
      * AlgorithmFactory.
      **/
      jobject get_java_instance (void);

      protected:
        /// the Java object with callable methods
        jobject obj_;
    };
  }
}

#endif // _GAMS_ALGORITHM_JAVA_H_
