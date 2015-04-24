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

import com.madara.KnowledgeBase;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportType;
import com.madara.transport.filters.LogAggregate;

import com.gams.controllers.BaseController;
import com.gams.utility.Logging;

public class TestDebuggerLoopFilters
{ 
  public static void main (String...args) throws Exception
  {
    //Logging logger = new Logging();
    //Logging.setLevel(10);
  	
    System.out.println("Creating QoS Transport Settings...");
    QoSTransportSettings transportSettings = new QoSTransportSettings();
    
    System.out.println("Adding LogAggregate() to settings...");
    transportSettings.addSendFilter(new LogAggregate());
    System.out.println("Adding multicast to settings...");
    transportSettings.setHosts(new String[]{"239.255.0.1:4150"});
    transportSettings.setType(TransportType.MULTICAST_TRANSPORT);
    
    System.out.println("Creating knowledge base...");
    KnowledgeBase knowledge = new KnowledgeBase("", transportSettings);
    System.out.println("Passing knowledge base to base controller...");
    BaseController controller = new BaseController(knowledge);
    
    System.out.println("Running controller every 1s for 10s...");
    controller.run(1.0, 10.0);
    
    System.out.println("Printing resulting knowledges...");
    knowledge.print();
    
    controller.free();
    transportSettings.free();
    knowledge.free();
  }
  
  
}