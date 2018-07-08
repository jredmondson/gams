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

package ai.gams.tests;

import ai.gams.algorithms.DebuggerAlgorithm;
import ai.gams.controllers.BaseController;
import ai.gams.exceptions.GamsDeadObjectException;
import ai.gams.platforms.DebuggerPlatform;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.TransportType;

public class TestMultiController
{
	static private class ControllerThread extends Thread
	{
		KnowledgeBase knowledge;
		BaseController controller;
		int id;
		int processes;
		double hertz;
		double length;

		public ControllerThread(int tid, int tprocesses, double thertz, double tlength, boolean networked)
				throws GamsDeadObjectException
		{
			if (networked)
			{
				QoSTransportSettings settings = new QoSTransportSettings();
				settings.setHosts(new String[]
				{ "239.255.0.1:4150" });
				settings.setType(TransportType.MULTICAST_TRANSPORT);

				knowledge = new KnowledgeBase("", settings);
			} else
			{
				knowledge = new KnowledgeBase();
			}

			controller = new BaseController(knowledge);

			id = tid;
			processes = tprocesses;
			hertz = thertz;
			length = tlength;

			System.out.println("Initializing vars in controller " + id + "...");
			controller.initVars((long) id, (long) processes);
			controller.initPlatform(new DebuggerPlatform());
			controller.initAlgorithm(new DebuggerAlgorithm());
		}

		public void run()
		{
			try
			{
				System.out.println("Running controller " + id + " at " + hertz + "hz for " + length + "s...");
				controller.runHz(hertz, length, -1);

				System.out.println("Finished running controller " + id + "...");
				System.out.println("Printing controller " + id + "'s knowledge...");
				knowledge.print();
				controller.free();
				knowledge.free();
			} catch (Exception e)
			{
				e.printStackTrace();
			}
		}
	}

	public static void main(String... args) throws Exception
	{
		int numControllers = 1;
		double hertz = 50;
		double length = 120;
		boolean networked = false;

		if (args.length > 0)
		{
			try
			{
				numControllers = Integer.parseInt(args[0]);
			} catch (NumberFormatException e)
			{
				System.err.println("Argument 1 (" + args[0] + ") is supposed to be number of controllers to run.");
				System.exit(1);
			}

			try
			{
				hertz = Double.parseDouble(args[1]);
			} catch (NumberFormatException e)
			{
				System.err.println("Argument 2 (" + args[1] + ") is supposed to be the hertz rate.");
				System.exit(1);
			}

			try
			{
				length = Double.parseDouble(args[2]);
			} catch (NumberFormatException e)
			{
				System.err.println("Argument 3 (" + args[2] + ") is supposed to be the length in seconds.");
				System.exit(1);
			}

			if (args.length >= 4)
			{
				if (args[3].equalsIgnoreCase("yes"))
				{
					networked = true;
				}
			}

		} else
		{
			System.err.println("Test takes four arguments: num_controls hertz length [networking?]");
			System.err.println("  num_controls specifies the number of controllers");
			System.err.println("  hertz specifies the hertz rate to run at");
			System.err.println("  length specifies the time in seconds to run");
			System.err.println("  networking is optional. By default the controllers"
					+ " are not networked. Any variation of yes will enable networking.");
			System.exit(1);
		}

		if (!networked)
		{
			System.out.println("Creating " + numControllers + " base controllers...");
		} else
		{
			System.out.println("Creating " + numControllers + " networked base controllers...");
		}

		ControllerThread[] controllers = new ControllerThread[numControllers];

		for (int i = 0; i < numControllers; ++i)
		{
			controllers[i] = new ControllerThread(i, numControllers, hertz, length, networked);
		}

		System.out.println("Starting " + numControllers + " base controllers...");

		for (int i = 0; i < numControllers; ++i)
		{
			controllers[i].start();
		}

		System.out.println("Waiting on " + numControllers + " base controllers...");

		for (int i = 0; i < numControllers; ++i)
		{
			controllers[i].join();
		}

	}

}