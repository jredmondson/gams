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
package ai.gams.variables;

import java.util.AbstractMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import ai.gams.exceptions.GamsDeadObjectException;

public class SensorMap extends AbstractMap<java.lang.String, Sensor>
{
	private native void jni_freeSensorMap(long[] ptrs, int length);

	private Set<Map.Entry<java.lang.String, Sensor>> mySet;
	private final boolean freeable;

	/**
	 * Base constructor for a sensor map
	 *
	 * @param keys
	 *            list of keys
	 * @param vals
	 *            list of values
	 **/
	public SensorMap(java.lang.String[] keys, long[] vals)
	{
		this(keys, vals, true);
	}

	/**
	 * Constructor for a sensor map
	 *
	 * @param keys
	 *            list of keys
	 * @param vals
	 *            list of values
	 * @param freeable
	 *            if true, indicates that the list is able to be freed
	 **/
	public SensorMap(java.lang.String[] keys, long[] vals, boolean freeable)
	{
		this.freeable = freeable;

		if (keys == null || vals == null || keys.length != vals.length)
			return;

		mySet = new HashSet<Map.Entry<java.lang.String, Sensor>>();

		for (int x = 0; x < keys.length; x++)
		{
			mySet.add(new SensorMapEntry(keys[x], vals[x], freeable));
		}
	}

	/**
	 * Returns an entry set for iteration
	 *
	 * @see java.util.AbstractMap#entrySet ()
	 * @return the iterable entry set
	 */
	@Override
	public Set<Map.Entry<java.lang.String, Sensor>> entrySet()
	{
		return mySet;
	}

	/**
	 * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be
	 * called before an instance of gets garbage collected
	 */
	public void free()
	{
		if (!freeable)
			return;

		long[] ptrs = new long[mySet == null ? 0 : mySet.size()];
		int pos = 0;

		for (Map.Entry<java.lang.String, Sensor> entry : mySet)
		{
			ptrs[pos++] = entry.getValue().getCPtr();
		}

		jni_freeSensorMap(ptrs, ptrs.length);

		mySet = null;

	}

	/**
	 * Map entry for iteration of entry set
	 **/
	private static class SensorMapEntry implements Map.Entry<java.lang.String, Sensor>
	{
		private java.lang.String key;
		private Sensor record;

		/**
		 * Can only be called from SensorMap
		 **/
		private SensorMapEntry(java.lang.String key, long val, boolean isNew)
		{
			this.key = key;
			try
			{
				record = Sensor.fromPointer(val, isNew);
			} catch (GamsDeadObjectException e)
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		/**
		 * Gets the key of the entry
		 *
		 * @see java.util.Map.Entry#getKey ()
		 * @return the key
		 */
		public java.lang.String getKey()
		{
			return key;
		}

		/**
		 * Gets the value of the entry
		 *
		 * @see java.util.Map.Entry#getValue ()
		 * @return the sensor object (value)
		 */
		public Sensor getValue()
		{
			return record;
		}

		/**
		 * Set value is unsupported. This is a read-only entry.
		 *
		 * @see java.util.Map.Entry#setValue (java.lang.Object)
		 * @param value
		 *            the value, if this were settable
		 * @return the value
		 */
		public Sensor setValue(Sensor value)
		{
			throw new UnsupportedOperationException("This map does not allow modification");
		}
	}
}
