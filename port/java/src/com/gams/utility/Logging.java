/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.utility;

import com.gams.GamsJNI;

public abstract class Logging extends GamsJNI
{
  private static native void jni_set_level(int level);
  private static native int jni_get_level();

  public static void setLevel(int level)
  {
    jni_set_level(level);
  }
  
  public static int getLevel()
  {
    return jni_get_level();
  }
}

