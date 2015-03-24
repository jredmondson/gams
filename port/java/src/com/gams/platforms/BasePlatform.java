/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.platforms;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.gams.variables.Self;
import com.gams.variables.Platform;
import com.gams.variables.SensorMap;
import com.gams.controllers.BaseController;

/**
 * Base class that should be extended when creating a Java platform for
 * usage in the GAMS controller
 */
public abstract class BasePlatform extends GamsJNI implements PlatformInterface
{ 
  private native long jni_getKnowledgeBase(long cptr);
  private native long jni_getSelf(long cptr);
  private native long jni_getPlatformStatus(long cptr);

  /**
   * Initialize the platform with controller variables. Use this
   * method to synchronize user-defined platforms with the controller.
   * @param  controller the controller that will be running the platform loop
   **/
  public void init (BaseController controller)
  {
    controller.initVars (this);
    knowledge = KnowledgeBase.fromPointer(jni_getKnowledgeBase(getCPtr()),false);
    self = Self.fromPointer(jni_getSelf(getCPtr()),false);
    status = com.gams.variables.Platform.fromPointer(jni_getPlatformStatus(getCPtr()),false);
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
   * Self-identifying variables like id and device properties
   **/
  public Self self;
  
  /**
   * The status of the platform
   **/
  public Platform status;
}

