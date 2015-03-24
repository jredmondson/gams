/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.utility;

/**
 * A utility class that acts as a facade for a region
 **/
public class PrioritizedRegion extends Region
{
  private native long jni_PrioritizedRegion();
  private static native void jni_freePrioritizedRegion(long cptr);
  private native long jni_getPriority(long cptr); 
  private native void jni_setPriority(long cptr, long priority); 
  
  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public PrioritizedRegion()
  {
    setCPtr(jni_PrioritizedRegion());
  }

  /**
   * Gets the priority of the region
   * @return priority of the region
   **/
  public long getPriority()
  {
    return jni_getPriority(getCPtr());
  }
  
  /**
   * Sets the priority of the region
   * @param priority the priority of the region
   **/
  public void setPriority(long priority)
  {
    jni_setPriority(getCPtr(),priority);
  }
  
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static PrioritizedRegion fromPointer(long cptr)
  {
    PrioritizedRegion ret = new PrioritizedRegion();
    ret.manageMemory = true;
    ret.setCPtr(cptr);
    return ret;
  }
  
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @param shouldManage  if true, manage the pointer
   * @return a new java instance of the underlying pointer
   */
  public static PrioritizedRegion fromPointer(long cptr, boolean shouldManage)
  {
    PrioritizedRegion ret = new PrioritizedRegion();
    ret.manageMemory=shouldManage;
    ret.setCPtr(cptr);
    return ret;
  }

  /**
   * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be
   * called before an instance of WaitSettings gets garbage collected
   */
  public void free()
  {
    if(manageMemory)
    {
      jni_freePrioritizedRegion(getCPtr());
    }
  }                
}

