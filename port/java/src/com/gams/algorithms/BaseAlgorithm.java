/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.algorithms;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.gams.platforms.BasePlatform;
import com.gams.variables.Self;
import com.gams.variables.Algorithm;
import com.gams.controllers.BaseController;

/**
 * Base class that should be extended when creating a Java algorithm for
 * usage in the GAMS controller
 */
public abstract class BaseAlgorithm extends GamsJNI implements AlgorithmInterface
{
  private native long jni_getKnowledgeBase(long cptr);
  private native long jni_getSelf(long cptr);
  private native Object jni_getPlatformObject(long cptr);
  private native long jni_getAlgorithmStatus(long cptr);

  /**
   * Initialize the platform with controller variables. Use this
   * method to synchronize user-defined algorithms with the controller.
   * @param  controller   controller which will be using the algorithm
   **/
  public void init (BaseController controller)
  {
    controller.initVars (this);
    platform = (BasePlatform)jni_getPlatformObject(getCPtr());
    knowledge = KnowledgeBase.fromPointer(jni_getKnowledgeBase(getCPtr()),false);
    self = Self.fromPointer(jni_getSelf(getCPtr()),false);
    status = Algorithm.fromPointer(jni_getAlgorithmStatus(getCPtr()),false);
  }
  
  /**
   * Facade for the protected setCPtr method in GamsJNI
   * @param cptr the C pointer for the underlying class
   **/
  public void assume (long cptr)
  {
    setCPtr(cptr);
  }
  
  /**
   * The controller's current knowledge base
   **/
  public KnowledgeBase knowledge;
  
  /**
   * The platform currently in use by the controller
   **/
  public BasePlatform platform;
  
  /**
   * Self-identifying variables like id and device properties
   **/
  public Self self;
  
  /**
   * The status of the algorithm
   **/
  public Algorithm status;
}

