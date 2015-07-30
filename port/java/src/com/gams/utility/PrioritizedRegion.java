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
package com.gams.utility;

/**
 * A utility class that acts as a facade for a region
 **/
public class PrioritizedRegion extends Region
{
  private native long jni_PrioritizedRegion();
  private native String jni_toString(long cptr);
  private native void jni_fromContainer(long cptr, long kb, String name);
  private native void jni_toContainer(long cptr, long kb, String name);
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
   * Get String representation of the PrioritizedRegion
   * @return String representation of this PrioritizedRegion
   **/
  public String toString()
  {
    return jni_toString(getCPtr());
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
   * Helper function for copying values from a MADARA knowledge base
   * @param kb      KnowledgeBase to copy region infomration to
   **/
  public void fromContainer(com.madara.KnowledgeBase kb)
  {
    fromContainer(kb, getName());
  }

  /**
   * Helper function for copying values from a MADARA knowledge base
   * @param kb      KnowledgeBase to copy region infomration to
   * @param name    name of the prioritized region
   **/
  public void fromContainer(com.madara.KnowledgeBase kb, String name)
  {
    jni_fromContainer(getCPtr(), kb.getCPtr(), name); 
  }

  /**
   * Helper function for copying values to a MADARA knowledge base
   * @param kb      KnowledgeBase with region information
   **/
  public void toContainer(com.madara.KnowledgeBase kb)
  {
    toContainer(kb, getName());
  }

  /**
   * Helper function for copying values to a MADARA knowledge base
   * @param kb      KnowledgeBase with region information
   * @param name    name of the prioritized region
   **/
  public void toContainer(com.madara.KnowledgeBase kb, String name)
  {
    jni_toContainer(getCPtr(), kb.getCPtr(), name);
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

