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
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *********************************************************************/

package com.gams.tests;
 
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import com.gams.controllers.BaseController;
import com.gams.algorithms.MessageProfiling;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeList;
import com.madara.transport.TransportSettings;
import com.madara.transport.filters.Packet;
import com.madara.transport.TransportContext;
import com.madara.Variables;

public class TestMessageProfilingAlgorithm
{
  public String host;
  public long duration;
  public double rate;

  public TestMessageProfilingAlgorithm ()
  {
    host = "";
    duration = 10;
    rate = 2;
  }

  public void test ()
  {
    TransportSettings settings = new TransportSettings ();
    KnowledgeBase knowledge = new KnowledgeBase (host, settings);
    BaseController controller = new BaseController (knowledge);
    //controller.initPlatform ("null", new KnowledgeList (new long[0]));
    controller.initPlatform ("null");
    controller.initAlgorithm (new MessageProfiling (controller));
    com.madara.logger.GlobalLogger.setLevel(6);
    controller.run (1.0 / rate, duration);
    com.madara.logger.GlobalLogger.setLevel(0);
  }

  public static void parseArgs (String[] args, TestMessageProfilingAlgorithm obj)
  {
    for (int i = 0; i < args.length; ++i)
    {
      if (args[i] == "-h" || args[i] == "--host")
      {
        obj.host = args[i + 1];
      }
      else if (args[i] == "-d" || args[i] == "--duration")
      {
        obj.duration = Long.parseLong (args[i + 1]);
      }
      else if (args[i] == "-r" || args[i] == "--rate")
      {
        obj.rate = Double.parseDouble (args[i + 1]);
      }
      else
      {
        System.err.println ("Invalid argument: " + args[i]);
        System.exit (-1);
      }
      ++i;
    }
  }

  public static void main (String[] args)
  {
    TestMessageProfilingAlgorithm obj = new TestMessageProfilingAlgorithm ();
    parseArgs (args, obj);
    obj.test ();
  }
}
