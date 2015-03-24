/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.platforms;

/**
 * Status of the platform
 */
public enum Status
{
  //These are defined in platforms/Base.h
  UNKNOWN(0),
  OK(1),
  WAITING(2),
  DEADLOCKED(4),
  FAILED(8),
  MOVING(16),
  REDUCED_SENSING_AVAILABLE(128),
  REDUCED_MOVEMENT_AVAILABLE(256),
  COMMUNICATION_AVAILABLE(512),
  SENSORS_AVAILABLE(1024),
  MOVEMENT_AVAILABLE(2048);

  private int num;

  private Status(int num)
  {
    this.num = num;
  }

  /**
   * @return int value of this {@link com.gams.platforms.Status Status}
   */
  public int value()
  {
    return num;
  }

  /**
   * Converts an int to a {@link com.gams.platforms.Status Status}
   *
   * @param val value to convert
   * @return {@link com.gams.platforms.Status Status} or null if the int is invalid
   */
  public static Status getType(int val)
  {
    for (Status t : values())
    {
      if (t.value() == val)
        return t;
    }
    return null;
  }
}

