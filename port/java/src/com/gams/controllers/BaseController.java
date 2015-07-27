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
package com.gams.controllers;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeList;
import com.gams.algorithms.BaseAlgorithm;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.DebuggerPlatform;
import com.gams.algorithms.DebuggerAlgorithm;
import com.gams.algorithms.AlgorithmFactory;

public class BaseController extends GamsJNI
{	
  private native long jni_BaseControllerFromKb(long cptr);
  private native long jni_BaseController(long cptr);
  private static native void jni_freeBaseController(long cptr);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_analyze(long cptr);
  private native long jni_execute(long cptr);
  private native long jni_getPlatform(long cptr);
  private native long jni_getAlgorithm(long cptr);
  private native void jni_addAlgorithmFactory(long cptr, java.lang.String name, Object factory);
  private native void jni_initAccent(long cptr, java.lang.String name, long[] args);
  private native void jni_initAlgorithm(long cptr, java.lang.String name, long[] args);
  private native void jni_initPlatform(long cptr, java.lang.String name);
  private native void jni_initPlatform(long cptr, java.lang.String name, long[] args);
  private native void jni_initAlgorithm(long cptr, Object algorithm);
  private native void jni_initPlatform(long cptr, Object platform);
  private native void jni_initVars(long cptr, long id, long processes);
  private native void jni_initVarsAlgorithm(long cptr, long algorithm);
  private native void jni_initVarsPlatform(long cptr, long platform);
  private native long jni_monitor(long cptr);
  private native long jni_plan(long cptr);
  private native long jni_run(long cptr, double period, double max);
  private native long jni_run(long cptr, double loopPeriod, double max, double sendPeriod);
  private native long jni_runHz(long cptr, double loopHz, double max, double sendHz);
  private native long jni_systemAnalyze(long cptr);

  private BaseAlgorithm  algorithm = null;
  private BasePlatform   platform = null;
  private long id = 0;
  private long processes = 1;
  
  private boolean manageMemory = true;

  /**
   * Constructor from C pointers
   * @param cptr the C-style pointer to the Base_Controller class instance
   */
  private BaseController(long cptr)
  {
    setCPtr(cptr);
  }
  
  /**
   * Constructor
   * @param knowledge knowledge base to use
   */
  public BaseController(KnowledgeBase knowledge)
  {
    setCPtr(jni_BaseControllerFromKb(knowledge.getCPtr ()));
    
    initVars(0, 1);
    initPlatform(new DebuggerPlatform ());
    initAlgorithm(new DebuggerAlgorithm ());
  }

  /**
   * Copy constructor
   * @param input the instance to copy 
   */
  public BaseController(BaseController input)
  {
    setCPtr(jni_BaseController(input.getCPtr()));
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @param shouldManage  if true, manage the pointer
   * @return a new java instance of the underlying pointer
   **/
  public static BaseController fromPointer(long cptr, boolean shouldManage)
  {
    BaseController ret = new BaseController(cptr);
    ret.manageMemory=shouldManage;
    return ret;
  }

  /**
   * Analyzes the platform, algorithm and accents
   * @return status of analyze. 0 is full success, though this is not
   *         currently checked.
   **/
  public long analyze ()
  {
    return jni_analyze(getCPtr());
  }
  
  /**
   * Executes the algorithm and accents
   * @return status of execute. 0 is full success, though this is not
   *         currently checked.
   **/
  public long execute ()
  {
    return jni_execute(getCPtr());
  }
  
  /**
   * Initialize an accent within the controller
   *
   * @param  name       name of the accent
   * @param  args       arguments to the accent initialization
   */
  public void initAccent(java.lang.String name, KnowledgeList args)
  {
    jni_initAccent(getCPtr(), name, args.toPointerArray());
  }

  /**
   * Initialize an algorithm within the controller
   *
   * @param  name       name of the algorithm
   * @param  args       arguments to the algorithm initialization
   */
  public void initAlgorithm(java.lang.String name, KnowledgeList args)
  {
    jni_initAlgorithm(getCPtr(), name, args.toPointerArray());
  }

  /**
   * Adds a named algorithm factory to the controller, which allows for usage
   * of the named factory in swarm.command and device.*.command
   * @param name   the string name to associate with the factory
   * @param factory the factory that creates the algorithm
   */
  public void addAlgorithmFactory(
    java.lang.String name, AlgorithmFactory factory)
  {
    jni_addAlgorithmFactory(getCPtr(), name, factory);
  }
  
  /**
   * Initialize a platform within the controller
   *
   * @param  name       name of the platform
   */
  public void initPlatform(java.lang.String name)
  {
    jni_initPlatform(getCPtr(), name);
  }

  /**
   * Initialize a platform within the controller
   *
   * @param  name       name of the platform
   * @param  args       arguments to the platform initialization
   */
  public void initPlatform(java.lang.String name, KnowledgeList args)
  {
    jni_initPlatform(getCPtr(), name, args.toPointerArray());
  }

  /**
   * Initialize an algorithm within the controller
   *
   * @param  algorithm  the algorithm to add to the controller
   */
  public void initAlgorithm(BaseAlgorithm algorithm)
  {
    this.algorithm = algorithm;
    jni_initAlgorithm(getCPtr(), algorithm);
    algorithm.assume(jni_getAlgorithm(getCPtr()));
    algorithm.init(this);
  }

  /**
   * Initialize a platform within the controller
   *
   * @param  platform  platform to add to the controller
   */
  public void initPlatform(BasePlatform platform)
  {
    this.platform = platform;
    jni_initPlatform(getCPtr(), platform);
    platform.assume(jni_getPlatform(getCPtr()));
    platform.init(this);
  }

  /**
   * Initialize the variables within the controller
   *
   * @param  id         the id of this process within the swarm
   * @param  processes  the number of processes participating in the swarm
   */
  public void initVars(long id, long processes)
  {
    this.id = id;
    this.processes = processes;
    jni_initVars(getCPtr(), id, processes);
    
    // if the user setup the platform, initialize with the new id
    if(platform != null)
    {
      platform.init(this); 
    }
    
    // if the user setup the platform, initialize with the new id
    if(algorithm != null)
    {
      algorithm.init(this); 
    }
  }

  /**
   * Initialize the variables for a platform. This function is useful
   * for user-defined platforms and essentially shares the controller's
   * KnowledgeBase, self-identifying variables, etc.
   *
   * @param  platform   the user-defined platform
   */
  public void initVars(BasePlatform platform)
  {
    jni_initVarsPlatform(getCPtr(), platform.getCPtr ());
  }

  /**
   * Initialize the variables for an algorithm. This function is useful
   * for user-defined algorithms and essentially shares the controller's
   * KnowledgeBase, self-identifying variables, etc.
   *
   * @param  algorithm   the user-defined algorithm
   */
  public void initVars(BaseAlgorithm algorithm)
  {
    jni_initVarsAlgorithm(getCPtr(), algorithm.getCPtr ());
  }

  /**
   * Monitors the platform's sensors
   * @return status of monitor. 0 is full success, though this is not
   *         currently checked.
   **/
  public long monitor ()
  {
    return jni_monitor(getCPtr());
  }
  
  /**
   * Plans the algorithm and accents
   * @return status of plan. 0 is full success, though this is not
   *         currently checked.
   **/
  public long plan ()
  {
    return jni_plan(getCPtr());
  }
 
  /**
   * Runs the monitor, analyze, plan and execute loop with a
   * specific period and duration
   * @param  period   the time in between executions of the loop in seconds
   * @param  duration the duration of the loop in seconds
   * @return status of run. 0 is full success, though this is not
   *         currently checked.
   **/
  public long run (double period, double duration)
  {
    return jni_run(getCPtr(), period, duration);
  }
   
  /**
   * Runs iterations of the MAPE loop with specified periods between loops
   * @param  loopPeriod time in seconds between executions of the loop. 0
   *                    means run as fast as possible (no sleeps).
   * @param  duration   the duration of time spent running the loop in seconds.
   *                    Negative duration means run loop forever. 0 duration
   *                    means run once.
   * @param  sendPeriod time in seconds between sending updates over network.
   *                    If sendPeriod is non-positive, loopPeriod is used.
   * @return status of run. 0 is full success, though this is not
   *         currently checked.
   **/
  public long run (double loopPeriod, double duration, double sendPeriod)
  {
    return jni_run(getCPtr(), loopPeriod, duration, sendPeriod);
  }
   
  /**
   * Runs iterations of the MAPE loop with specified hertz
   * @param  loopHz   the intended hz at which the loop should execute.
   *                  anything non-negative is valid. 0hz is treated as
   *                  as infinite hertz (i.e., run as fast as possible).
   * @param  duration the duration of time spent running the loop in seconds.
   *                  Negative duration means run loop forever. 0 duration means
   *                  run once.
   * @param  sendHz   the intended hz at which updates should be sent. If
   *                  non-positive, loopHz is used.
   * @return  the result of the MAPE loop
   * @return status of run. 0 is full success, though this is not
   *         currently checked.
   **/
  public long runHz (double loopHz, double duration, double sendHz)
  {
    return jni_runHz(getCPtr(), loopHz, duration, sendHz);
  }
   
  /**
   * Analyzes the controller and system
   * @return status of analyze. 0 is full success, though this is not
   *         currently checked.
   **/
  public long systemAnalyze ()
  {
    return jni_systemAnalyze(getCPtr());
  }
  
  /**
   * Converts the value to a string
   *
   * @return current string value
   */
  public java.lang.String toString()
  {
    return "BaseController";
  }

  /**
   * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be
   * called before an instance of WaitSettings gets garbage collected
   */
  public void free()
  {
    if (manageMemory)
    {
      jni_freeBaseController(getCPtr());
    }
    setCPtr(0);
  }
  
  /**
   * Cleans up underlying C resources
   * @throws Throwable necessary for override but unused
   */
  @Override
  protected void finalize() throws Throwable
  {
    try {
      free();
    } catch (Throwable t) {
      throw t;
    } finally {
      super.finalize();
    }
  }
}

