/*********************************************************************
 * Copyright (c) 2013-2015 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following acknowledgments and disclaimers.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or
 * "Software Engineering Institute" shall not be used to endorse or promote
 * products derived from this software without prior written permission. For
 * written permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 * appear in their names without prior written permission of
 * permission@sei.cmu.edu.
 *
 * 5. Redistributions of any form whatsoever must retain the following
 * acknowledgment:
 *
 * This material is based upon work funded and supported by the Department of
 * Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon University
 * for the operation of the Software Engineering Institute, a federally funded
 * research and development center. Any opinions, findings and conclusions or
 * recommendations expressed in this material are those of the author(s) and
 * do not necessarily reflect the views of the United States Department of
 * Defense.
 * 
 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 * INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 * UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 * AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR
 * PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE
 * MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND
 * WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 * This material has been approved for public release and unlimited
 * distribution.
 * 
 * @author James Edmondson <jedmondson@gmail.com>
 *********************************************************************/
package com.gams.algorithms;

import com.madara.containers.Integer;

/**
 * Algorithm settings debugger class
 */
public class DebuggerAlgorithm extends BaseAlgorithm
{
  /**
   * Default constructor
   **/
  public DebuggerAlgorithm()
  {
    executions = new com.madara.containers.Integer();
  }

  /**
   * Analyzes the state of the algorithm
   **/
  public int analyze()
  {
    executions.setName(knowledge, ".executions");
    System.out.println(self.id.get() + ":" + executions.get () +
      ":Algorithm.analyze called");
    
    if(platform.getCPtr() != 0)
    {
      System.out.println(self.id.get() + ":" + executions.get () +
        ":  platform_id: " + platform.getId() +
      	", platform_name: " + platform.getName());
      platform.getPositionAccuracy();
      platform.getPosition();
      platform.getMinSensorRange();
      platform.getMoveSpeed();
    }
    else
    {
      System.out.println(self.id.get() + ":" + executions.get () +
        ":  platform is null, so no operations called");
    }
    
    return AlgorithmStatusEnum.OK.value();
  }
  
  /**
   * Plans the next stage of the algorithm
   **/
  public int plan()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":Algorithm.plan called");
    
    return AlgorithmStatusEnum.OK.value();
  }
  
  /**
   * Executes the next stage of the algorithm
   **/
  public int execute()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":Algorithm.execute called");
    
    if(platform.getCPtr() != 0)
    {
      platform.takeoff();
      platform.home();
      platform.land();
      platform.move(platform.getPosition(), 2.0);
      platform.setMoveSpeed(3.0);
      platform.stopMove();
    }
    else
    {
      System.out.println(self.id.get() + ":" + executions.get () +
        ":  platform is null, so no operations called");
    }
    executions.inc();
    
    return AlgorithmStatusEnum.OK.value();
  }
  
  private com.madara.containers.Integer executions;
}

