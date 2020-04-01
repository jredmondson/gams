/**
 * Copyright(c) 2020 Galois. All Rights Reserved.
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
 * 3. The names "Galois," "Carnegie Mellon University," "SEI" and/or "Software
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
 * @file test_hivecontroller.cpp
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a test driver for the GAMS HiveController.
 **/

#include "madara/knowledge/Hive.h"
#include "gams/controllers/HiveController.h"
#include "madara/logger/GlobalLogger.h"

// create shortcuts to MADARA classes and namespaces
namespace knowledge                = madara::knowledge;
namespace controllers              = gams::controllers;
typedef knowledge::KnowledgeRecord   KnowledgeRecord;
typedef KnowledgeRecord::Integer     Integer;

int gams_fails = 0;

void test_hivecontroller(void)
{
  knowledge::Hive hive;
  hive.resize(1000);

  std::vector<knowledge::KnowledgeBase>& kbs = hive.get_kbs();

  if (kbs.size() != hive.get_size())
  {
    std::cerr << "ERROR: hive size is incorrect.\n";
    gams_fails++;
  }
  
  controllers::HiveController controller1(hive, 0, 100);
  controllers::HiveController controller2(hive, 100, 100);
  controllers::HiveController controller3(hive, 200, 100);
  controllers::HiveController controller4(hive, 300, 300);
  controllers::HiveController controller5(hive, 600, 200);
  
  if (controller1.get_num_controllers() != 100)
  {
    std::cerr << "ERROR: controller1.get_num_controllers() is incorrect.\n";
    gams_fails++;
  }
  
  if (controller2.get_num_controllers() != 100)
  {
    std::cerr << "ERROR: controller2.get_num_controllers() is incorrect.\n";
    gams_fails++;
  }
  
  if (controller3.get_num_controllers() != 100)
  {
    std::cerr << "ERROR: controller3.get_num_controllers() is incorrect.\n";
    gams_fails++;
  }
  
  if (controller4.get_num_controllers() != 300)
  {
    std::cerr << "ERROR: controller4.get_num_controllers() is incorrect.\n";
    gams_fails++;
  }
  
  if (controller5.get_num_controllers() != 200)
  {
    std::cerr << "ERROR: controller5.get_num_controllers() is incorrect.\n";
    gams_fails++;
  }
  
  controllers::HiveController controller6, controller7;
  controllers::HiveController controller8, controller9;
  
  if (controller6.get_num_controllers() != 0 ||
      controller7.get_num_controllers() != 0 ||
      controller8.get_num_controllers() != 0 ||
      controller9.get_num_controllers() != 0)
  {
    std::cerr << "ERROR: controller6-9 init sizes are incorrect.\n";
    gams_fails++;
  }
  
  controller6.set_hive(hive);
  controller6.resize(800, 100);
  controller7.set_hive(hive);
  controller7.resize(900, 50);
  controller8.set_hive(hive);
  controller8.resize(950, 25);
  controller9.set_hive(hive);
  controller9.resize(975, 25);

  
  if (controller6.get_num_controllers() != 100 ||
      controller7.get_num_controllers() != 50 ||
      controller8.get_num_controllers() != 25 ||
      controller9.get_num_controllers() != 25)
  {
    std::cerr << "ERROR: controller6-9 resize sizes are incorrect.\n";
    gams_fails++;
  }

  controller9.set_hive(hive);
  controller9.resize(975, 100);
  
  if (controller9.get_num_controllers() != 25)
  {
    std::cerr << "ERROR: controller9 resize should have corrected to 25.\n";
    gams_fails++;
  }

}

// perform main logic of program
int main(int argc, char ** argv)
{
  
  
  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}
