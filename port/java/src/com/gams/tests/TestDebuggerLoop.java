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
import com.gams.controllers.BaseController;
import com.gams.utility.Logging;

public class TestDebuggerLoop
{ 
  public static void main (String...args) throws Exception
  {
    //Logging logger = new Logging();
    //Logging.setLevel(10);
  	  
    System.out.println("Creating knowledge base...");
    KnowledgeBase knowledge = new KnowledgeBase();
    System.out.println("Passing knowledge base to base controller...");
    BaseController controller = new BaseController(knowledge);
    
    System.out.println("Running controller every 1s for 10s...");
    controller.run(1.0, 200.0);
    
    controller.free();
    knowledge.free();
  }
  
  
}