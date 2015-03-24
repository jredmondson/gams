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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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
 * @file VREP_Ant.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the VREP_Ant simulator ant robot class
 */

#ifdef _GAMS_VREP_ // only compile this if we are simulating in VREP

#include "gams/platforms/vrep/VREP_Ant.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

#include <iostream>
using std::endl;
using std::cout;
using std::string;
#include <cmath>

#include "madara/knowledge_engine/containers/Double_Vector.h"

#include "gams/variables/Sensor.h"

gams::platforms::VREP_Ant::VREP_Ant (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : VREP_Base (knowledge, sensors, self)
{
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  self_->device.desired_altitude = 0.05;
  add_model_to_environment ();
  set_initial_position ();
  get_target_handle ();
  wait_for_go ();
}

void
gams::platforms::VREP_Ant::add_model_to_environment ()
{
  string modelFile (getenv ("GAMS_ROOT"));
  modelFile += "/resources/vrep/tracker_ant.ttm";
  if (simxLoadModel (client_id_, modelFile.c_str (), 0, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    cerr << "error loading VREP_Ant model in vrep" << endl;
    exit (-1);
  }

  if (node_id_ < 0)
  {
    cerr << "invalid handle id for VREP_Ant base" << endl;
    exit (-1);
  }
}

std::string
gams::platforms::VREP_Ant::get_id () const
{
  return "vrep_ant";
}

std::string
gams::platforms::VREP_Ant::get_name () const
{
  return "VREP Ant";
}

void
gams::platforms::VREP_Ant::get_target_handle ()
{
  simxGetObjectChild(client_id_, node_id_, 0, &node_target_, simx_opmode_oneshot_wait);

  if (node_target_ < 0)
  {
    cerr << "invalid handle id for VREP_Ant base" << endl;
    exit (-1);
  }
}

#endif // _GAMS_VREP_
