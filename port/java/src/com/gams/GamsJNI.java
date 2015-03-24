/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams;

/**
 * Abstract class that insures loading of libMADARA.so and holds the C pointer
 */
public abstract class GamsJNI
{
  static
  {
    System.loadLibrary("ACE");
    System.loadLibrary("MADARA");
    System.loadLibrary("GAMS");
  }

  /**
   * C pointer to an object
   */
  private long cptr = 0;

  /**
   * Set the C pointer to the object
   *
   * @param cptr C Pointer
   */
  protected void setCPtr(long cptr)
  {
    this.cptr = cptr;
  }


  /**
   * @return The C pointer of this object for passing to JNI functions
   */
  public long getCPtr()
  {
    return cptr;
  }


  /**
   * @return &lt;ClassName&gt;[&lt;C Pointer&gt;]
   * @see java.lang.Object#toString ()
   */
  public String toString()
  {
    return getClass().getName() + "[" + cptr + "]";
  }
}

