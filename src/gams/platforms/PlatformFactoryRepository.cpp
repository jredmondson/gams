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

#include "madara/utility/Utility.h"

#include "PlatformFactoryRepository.h"
#include "DebugPlatform.h"
#include "NullPlatform.h"

#ifdef _GAMS_VREP_
#include "gams/platforms/vrep/VREPQuad.h"
#include "gams/platforms/vrep/VREPQuadLaser.h"
#include "gams/platforms/vrep/VREPQuadCDRA.h"
#include "gams/platforms/vrep/VREPAnt.h"
#include "gams/platforms/vrep/VREPSummit.h"
#include "gams/platforms/vrep/VREPBoat.h"
#endif

#ifdef _GAMS_ROS_
#include "gams/platforms/ros/RosP3Dx.h"
#endif 

#include "gams/loggers/GlobalLogger.h"

#include <string>

namespace platforms = gams::platforms;
namespace variables = gams::variables;

GAMSExport platforms::PlatformFactoryRepository *
    platforms::global_platform_factory() {
  static platforms::PlatformFactoryRepository *plat_repo =
    new platforms::PlatformFactoryRepository();
  return plat_repo;
}

platforms::PlatformFactoryRepository::PlatformFactoryRepository (
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
: knowledge_ (knowledge), platforms_ (platforms), self_ (self),
  sensors_ (sensors)
{
}

platforms::PlatformFactoryRepository::~PlatformFactoryRepository ()
{
}

void
platforms::PlatformFactoryRepository::initialize_default_mappings (void)
{
  std::vector <std::string> aliases;
  
  // the debug platform
  aliases.resize (3);
  aliases[0] = "debug";
  aliases[1] = "print";
  aliases[2] = "printer";

  add (aliases, new DebugPlatformFactory ());
  
  // the null platform
  aliases.resize (1);
  aliases[0] = "null";

  add (aliases, new NullPlatformFactory ());
  
  // VREP Platforms
#ifdef _GAMS_VREP_
  
  // the VREP Ant platform
  aliases.resize (2);
  aliases[0] = "vrep-ant";
  aliases[1] = "vrep_ant";

  add (aliases, new VREPAntFactory ());
  
  // the VREP Quadcopter platform
  aliases.resize (4);
  aliases[0] = "vrep-uav";
  aliases[1] = "vrep_uav";
  aliases[2] = "vrep-quad";
  aliases[3] = "vrep_quad";

  add (aliases, new VREPQuadFactory ());

  // the VREP Quadcopter platform
  aliases.resize (4);
  aliases[0] = "vrep-uav-laser";
  aliases[1] = "vrep_uav-laser";
  aliases[2] = "vrep-quad-laser";
  aliases[3] = "vrep_quad-laser";

  add (aliases, new VREPQuadLaserFactory ());
  
  // the VREP Quadcopter CDRA platform
  aliases.resize (2);
  aliases[0] = "vrep-quad-cdra";
  aliases[1] = "vrep_quad_cdra";

  add (aliases, new VREPQuadCDRAFactory ());

  // the VREP Summit platform
  aliases.resize (2);
  aliases[0] = "vrep-summit";
  aliases[1] = "vrep_summit";

  add (aliases, new VREPSummitFactory ());
  
 // the VREP Boat platform
  aliases.resize (2);
  aliases[0] = "vrep-boat";
  aliases[1] = "vrep_boat";

  add (aliases, new VREPBoatFactory ());
#endif

  // ROS Platforms
#ifdef _GAMS_ROS_
  // the ROS P3DX platform
  aliases.resize (2);
  aliases[0] = "RosP3Dx";
  aliases[1] = "ros-p3dx";
  aliases[2] = "ROS_P3DX";

  add (aliases, new RosP3DxFactory ());
#endif
}

void
platforms::PlatformFactoryRepository::add (
  const std::vector <std::string> & aliases,
  PlatformFactory * factory)
{
  for (size_t i = 0; i < aliases.size (); ++i)
  {
    std::string alias (aliases[i]);
    madara::utility::lower (alias);

    factory->set_knowledge (knowledge_);
    factory->set_self (self_);
    factory->set_sensors (sensors_);
    factory->set_platforms (platforms_);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::PlatformFactoryRepository:add" \
      " Adding %s factory.\n", alias.c_str ());

    factory_map_[alias] = factory;
  }
}

platforms::BasePlatform *
platforms::PlatformFactoryRepository::create (
  const std::string & type,
  const madara::knowledge::KnowledgeMap & args)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MINOR,
    "gams::platforms::PlatformFactoryRepository::create(" \
    "%s,...)\n", type.c_str ());

  BasePlatform * result (0);

  if (type != "")
  {
    std::string alias (type);
    madara::utility::lower (alias);

    FactoryMap::iterator it = factory_map_.find (alias);
    if (it != factory_map_.end ())
    {
      result = it->second->create (args, knowledge_, sensors_, platforms_, 
        self_);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::PlatformFactoryRepository::create:" \
        " could not find \"%s\" platform.\n", alias.c_str ());
    }
  }

  return result;
}

void
platforms::PlatformFactoryRepository::set_knowledge (
  madara::knowledge::KnowledgeBase * knowledge)
{
  knowledge_ = knowledge;
}

void
platforms::PlatformFactoryRepository::set_platforms (
  variables::Platforms * platforms)
{
  platforms_ = platforms;
}

void
platforms::PlatformFactoryRepository::set_self (variables::Self * self)
{
  self_ = self;
}

void
platforms::PlatformFactoryRepository::set_sensors (
  variables::Sensors * sensors)
{
  sensors_ = sensors;
}
