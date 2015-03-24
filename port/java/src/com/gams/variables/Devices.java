/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.variables;

import java.util.AbstractList;

/**
 * Devices provides a read-only interface for devices
 */
public class Devices extends AbstractList<Device>
{
  private native void jni_freeDevices(long[] records, int length);

  private long[] devices;

  /**
   * Constructor
   * @param input list of C pointers to the underlying devices
   **/
  public Devices(long[] input)
  {
    devices = input;
  }

  /**
   * Gets the device at the specified index
   * @see java.util.AbstractList#get (int)
   * @param index the element of the device list to retrieve
   */
  @Override
  public Device get(int index)
  {
    return Device.fromPointer(devices[index]);
  }

  /**
   * Returns the size of the device list
   * @see java.util.AbstractCollection#size ()
   * @return the size of the device list
   */
  @Override
  public int size()
  {
    return devices == null ? 0 : devices.length;
  }


  /**
   * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be called
   * before an instance gets garbage collected
   */
  public void free()
  {
    if (devices == null)
      return;
    jni_freeDevices(devices, devices.length);
    devices = null;
  }
}

