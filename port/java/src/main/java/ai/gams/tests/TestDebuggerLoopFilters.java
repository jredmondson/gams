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
 
import ai.gams.controllers.BaseController;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.TransportType;
import ai.madara.transport.filters.LogAggregate;

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