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
 * @file test_vrep_controller.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains a test driver for the VREP platform code.
 **/

#include "gams/platforms/vrep/VREP_UAV.h"
#include "gams/utility/Position.h"

#include "madara/knowledge_engine/KnowledgeBase.h"
#include "madara/utility/Utility.h"

#include "gams/variables/Sensor.h"
#include "gams/variables/Platform.h"
#include "gams/variables/Self.h"

#include <iostream>
using std::cout;
using std::endl;

extern "C" {
#include "extApi.h"
}

void start_simulation(int port)
{
  // start simulation
  const char* const IP_ADDRESS = "127.0.0.1";
  int client_id = simxStart (IP_ADDRESS, port, true, true, 2000, 5);
  simxStartSimulation(client_id, simx_opmode_oneshot_wait);
}

// perform main logic of program
int main (int argc, char* argv[])
{
  int port = 19905;
  if(argc >= 2)
    port = atoi(argv[1]);
  cout << "using port: " << port << endl;
  Madara::KnowledgeEngine::KnowledgeBase knowledge;
  knowledge.set(".vrep_port", Madara::KnowledgeRecord::Integer(port));
  gams::variables::Sensors sensors;
  gams::variables::Platforms platform;
  gams::variables::Self self;

  cout << "creating uav" << endl;
  gams::platforms::VREP_UAV uav(knowledge, &sensors, platform, self);

  cout << "starting sim" << endl;
  start_simulation(port + 1);

  gams::utility::Position pos(1.0,1.0,1.0);

  cout << "move1" << endl;

  while(uav.move(pos))
    Madara::Utility::sleep (0.001);

  pos.x = -2.0; pos.y = -2.0;
  cout << "move2" << endl;

  while(uav.move(pos))
    Madara::Utility::sleep (0.001);

  pos.x = 3.0; pos.y = 3.0;
  cout << "move3" << endl;

  while(uav.move(pos))
    Madara::Utility::sleep (0.001);

  return 0;
}
