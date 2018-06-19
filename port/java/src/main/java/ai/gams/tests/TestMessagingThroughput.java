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

package ai.gams.tests;
 
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import ai.madara.knowledge.KnowledgeBase;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.filters.Packet;
import ai.madara.transport.TransportContext;
import ai.madara.knowledge.Variables;

public class TestMessagingThroughput
{
  public String type;
  public String host;
  public long duration;
  public double rate;

  public TestMessagingThroughput ()
  {
    host = "";
    duration = 10;
    rate = 2;
  }

  private class CounterFilter implements ai.madara.transport.filters.AggregateFilter
  {
    public int counter;

    public CounterFilter ()
    {
      counter = 0;
    }

    public void filter (Packet packet, TransportContext context, Variables variables)
    {
      ++counter;
    }
  }

  public void testReader ()
  {
    QoSTransportSettings settings = new QoSTransportSettings ();
    CounterFilter filter = new CounterFilter ();
    settings.addReceiveFilter (filter);

    final String default_multicast = new String ("239.255.0.1:4150");
    String[] hosts = new String[1];
    hosts[0] = default_multicast;
    settings.setHosts (hosts);
    settings.setType (ai.madara.transport.TransportType.MULTICAST_TRANSPORT);

    KnowledgeBase kb = new KnowledgeBase(host, settings);

    ai.madara.util.Utility.sleep (duration);

    ai.madara.logger.GlobalLogger.log (6, "Counter: " + filter.counter);
  }

  public void testWriter ()
  {
    QoSTransportSettings settings = new QoSTransportSettings ();

    final String default_multicast = new String ("239.255.0.1:4150");
    String[] hosts = new String[1];
    hosts[0] = default_multicast;
    settings.setHosts (hosts);
    settings.setType (ai.madara.transport.TransportType.MULTICAST_TRANSPORT);

    KnowledgeBase kb = new KnowledgeBase(host, settings);

    int iterations = (int) duration * (int) rate;
    for (int i = 0; i < iterations; ++i)
    {
      kb.set ("data", 1);
      ai.madara.util.Utility.sleep (1 / rate);
    }
  }

  public void testBoth ()
  {
    QoSTransportSettings settings = new QoSTransportSettings ();
    CounterFilter filter = new CounterFilter ();
    settings.addReceiveFilter (filter);

    final String default_multicast = new String ("239.255.0.1:4150");
    String[] hosts = new String[1];
    hosts[0] = default_multicast;
    settings.setHosts (hosts);
    settings.setType (ai.madara.transport.TransportType.MULTICAST_TRANSPORT);

    KnowledgeBase kb = new KnowledgeBase(host, settings);

    int iterations = (int) duration * (int) rate;
    for (int i = 0; i < iterations; ++i)
    {
      kb.set ("data", 1);
      ai.madara.util.Utility.sleep (1 / rate);
    }

    ai.madara.logger.GlobalLogger.log (6, "Counter: " + filter.counter);
  }

  public static void parseArgs (String[] args, TestMessagingThroughput obj)
  {
    for (int i = 0; i < args.length; ++i)
    {
      if (args[i] == "-t" || args[i] == "--type")
      {
        obj.type = args[i + 1];
      }
      else if (args[i] == "-h" || args[i] == "--host")
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
    TestMessagingThroughput obj = new TestMessagingThroughput ();
    parseArgs (args, obj);

    switch (obj.type)
    {
      case "reader":
        obj.testReader ();
        break;
      case "writer":
        obj.testWriter ();
        break;
      case "both":
        obj.testBoth ();
        break;
      default:
        System.out.println ("Invalid type specified: " + obj.type);
        System.exit (-1);
    }
  }
}
