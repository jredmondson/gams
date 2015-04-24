/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.controllers;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeList;
import com.gams.algorithms.BaseAlgorithm;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.DebuggerPlatform;
import com.gams.algorithms.DebuggerAlgorithm;

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
  private native void jni_initAccent(long cptr, java.lang.String name, long[] args);
  private native void jni_initAlgorithm(long cptr, java.lang.String name, long[] args);
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

  public BaseController(KnowledgeBase knowledge)
  {
    setCPtr(jni_BaseControllerFromKb(knowledge.getCPtr ()));
    
    initVars(0, 1);
    initPlatform(new DebuggerPlatform ());
    initAlgorithm(new DebuggerAlgorithm ());
  }

  public BaseController(BaseController input)
  {
    setCPtr(jni_BaseController(input.getCPtr()));
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
    jni_initVars(getCPtr(), id, processes);
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
    jni_freeBaseController(getCPtr());
    setCptr(0);
  }
}

