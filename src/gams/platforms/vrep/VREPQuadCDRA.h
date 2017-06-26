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
 * @file VREPQuadCDRA.h
 * @author Sagar Chaki <chaki@sei.cmu.edu>
 *
 * This file contains the definition of the VREPQuadCDRA simulator uav class
 **/

#ifndef   _GAMS_PLATFORM_VREP_QUAD_CDRA_H_
#define   _GAMS_PLATFORM_VREP_QUAD_CDRA_H_

#include "gams/platforms/PlatformFactory.h"
#include "gams/platforms/vrep/VREPBase.h"
#include "gams/platforms/vrep/VREPQuad.h"
#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/threads/Threader.h"
#include "madara/threads/BaseThread.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"

extern "C" {
#include "extApi.h"
}

#ifdef _GAMS_VREP_

namespace gams
{
  namespace platforms
  {
    /**
    * A VREP platform for an autonomous aerial quadcopter developed
    * for the CDRA project. This has the ability to interface with a
    * controller (Keyboard, XBOX360 etc.) and to send actuation
    * commands to individual motors of the underlying VREP model.
    **/
    class GAMSExport VREPQuadCDRA : public VREPQuad
    {
    public:
      const static std::string DEFAULT_MODEL;

      /**
       * Constructor
       * @param  file         model file to load
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge    knowledge base
       * @param  sensors      map of sensor names to sensor information
       * @param  platforms    map of platform names to platform information
       * @param  self         agent variables that describe self state
       **/
      VREPQuadCDRA (
        std::string model_file, 
        simxUChar is_client_side, 
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);
      
      /**
       * Gets the unique identifier of the platform. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. vrep_ant, autonomous_snake, etc.)
       **/
      virtual std::string get_id () const;

      /**
       * Gets the name of the platform
       **/
      virtual std::string get_name () const;
    }; // class VREPQuadCDRA

    /**
     * A factory class for creating VREP Quadcopter platforms
     **/
    class GAMSExport VREPQuadCDRAFactory : public VREPQuadFactory
    {
    protected:
      virtual std::string get_default_model();

      /**
       * Call through to VREPQuadCDRA Constructor
       * @param  file         model file to load
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge    knowledge base
       * @param  sensors      map of sensor names to sensor information
       * @param  platforms    map of platform names to platform information
       * @param  self         agent variables that describe self state
       * @return a new VREPQuadCDRA
       **/
      virtual VREPQuadCDRA *create_quad (
        std::string model_file, 
        simxUChar is_client_side, 
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);
    };
  } // namespace platform
} // namespace gams

#endif // _GAMS_VREP_

#endif // _GAMS_PLATFORM_VREP_QUAD_CDRA_H_
