/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/

package com.gams.tests;
 
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.lang.Integer;
import java.lang.Double;
import java.lang.Thread;

import com.madara.KnowledgeBase;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportType;
import com.gams.controllers.BaseController;
import com.gams.utility.Logging;
import com.gams.platforms.DebuggerPlatform;
import com.gams.algorithms.DebuggerAlgorithm;

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
    
    public ControllerThread (int tid, int tprocesses,
      double thertz, double tlength, boolean networked)
    {
      if (networked)
      {
        QoSTransportSettings settings = new QoSTransportSettings();
        settings.setHosts(new String[]{"239.255.0.1:4150"});
        settings.setType(TransportType.MULTICAST_TRANSPORT);
        
        knowledge = new KnowledgeBase("", settings);
      }
      else
      {
        knowledge = new KnowledgeBase();
      }
      
      controller = new BaseController(knowledge);
      
      id = tid;
      processes = tprocesses;
      hertz = thertz;
      length = tlength;
      
      System.out.println("Initializing vars in controller " + id + "...");
      controller.initVars((long)id, (long)processes);
      controller.initPlatform(new DebuggerPlatform ());
      controller.initAlgorithm(new DebuggerAlgorithm ());
    }
    
    public void run ()
    {
      System.out.println("Running controller " + id + " at " +
        hertz + "hz for " + length + "s...");
      controller.runHz(hertz, length, -1);
      
      System.out.println("Finished running controller " + id + "...");
      System.out.println("Printing controller " + id + "'s knowledge...");
      knowledge.print();
      controller.free();
      knowledge.free();
    }
  }
  
  
  public static void main (String...args) throws Exception
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
      }
      catch (NumberFormatException e)
      {
        System.err.println("Argument 1 (" + args[0] +
          ") is supposed to be number of controllers to run.");
        System.exit(1);
      }
      
      
      try
      {
        hertz = Double.parseDouble(args[1]);
      }
      catch (NumberFormatException e)
      {
        System.err.println("Argument 2 (" + args[1] +
          ") is supposed to be the hertz rate.");
        System.exit(1);
      }
      
      try
      {
        length = Double.parseDouble(args[2]);
      }
      catch (NumberFormatException e)
      {
        System.err.println("Argument 3 (" + args[2] +
          ") is supposed to be the length in seconds.");
        System.exit(1);
      }
      
      if (args.length >= 4)
      {
        if (args[3].equalsIgnoreCase("yes"))
        {
          networked = true;
        }
      }
      
    }
    else
    {
      System.err.println("Test takes four arguments: num_controls hertz length [networking?]");
      System.err.println("  num_controls specifies the number of controllers");
      System.err.println("  hertz specifies the hertz rate to run at");
      System.err.println("  length specifies the time in seconds to run");
      System.err.println("  networking is optional. By default the controllers" +
        " are not networked. Any variation of yes will enable networking.");
      System.exit(1);
    }

    if (!networked)
    {
      System.out.println("Creating " + numControllers + " base controllers...");
    }
    else
    {
      System.out.println("Creating " + numControllers +
        " networked base controllers...");
    }
    
    ControllerThread [] controllers = new ControllerThread[numControllers];
    
    for (int i = 0; i < numControllers; ++i)
    {
      controllers[i] = new ControllerThread(i, numControllers,
        hertz, length, networked);
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