package edu.cmu.gams.gamsmessageprofiling;

import android.content.Context;
import android.net.wifi.WifiManager;
import android.util.Log;
import android.os.Bundle;

import ai.gams.algorithms.MessageProfiling;
import ai.gams.controllers.BaseController;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.knowledge.KnowledgeList;
import ai.madara.knowledge.KnowledgeRecord;
import ai.madara.MadaraLog;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.TransportSettings;
import ai.madara.transport.TransportType;

/**
 * Created by aldukeman on 6/16/15.
 */
class Message_Profiler extends Thread
{
    private TransportType type;
    private String address;
    private int sendSize;
    private int rate;
    private int duration;
    private int id;
    private int swarmSize;

    private static final String LOCAL_TAG = "Message_Profiler";

    public Message_Profiler (String t, String a, int s, int r, int d, int i, int swarm)
    {
        switch(t)
        {
            case "Multicast":
                type = TransportType.MULTICAST_TRANSPORT;
                break;
            case "Broadcast":
                type = TransportType.BROADCAST_TRANSPORT;
                break;
            case "UDP":
                type = TransportType.UDP_TRANSPORT;
                break;
        }

        address = a;
        sendSize = s;
        rate = r;
        duration = d;
        id = i;
        swarmSize = swarm;
    }

    public void run ()
    {
        // Try C++ version of algorithm
        KnowledgeBase knowledge = new KnowledgeBase(Integer.toString(id), new TransportSettings());
        knowledge.set(".id", id);
        BaseController controller = new BaseController(knowledge);
        controller.initVars(id, swarmSize);
        controller.initPlatform("null");
        long[] records = new long[1];
        KnowledgeRecord s = new KnowledgeRecord(sendSize);
        records[0] = s.getCPtr();
        KnowledgeList args = new KnowledgeList(records);
        controller.initAlgorithm("message profiling", args);
        controller.run(1.0 / rate, duration);
        Log.d (LOCAL_TAG, knowledge.toString ());

        /*
        // Try Java version of algorithm
        String host = Integer.toString(id);
        KnowledgeBase knowledge = new KnowledgeBase(host, new TransportSettings());
        knowledge.set (".id", id);

        QoSTransportSettings settings = new QoSTransportSettings();
        String[] hosts = new String[1];
        hosts[0] = address;
        settings.setHosts(hosts);
        settings.setType(type);

        BaseController controller = new BaseController(knowledge);
        controller.initVars(id, 2);
        controller.initPlatform("null");
        MessageProfiling algo = new MessageProfiling();
        controller.initAlgorithm(algo);
        algo.initVars(settings);
        controller.run(1.0 / rate, duration);
        ai.madara.logger.GlobalLogger.log (6, knowledge.toString ());
        */


        /*
        // Try Java test version of algorithm
        try {
            String[] args = new String[2];
            args[0] = "--id";
            args[1] = Integer.toString(id);
            ai.gams.tests.TestMessageProfilingAlgorithm.main (args);
        } catch (Exception e) {
            e.printStackTrace();
        }
        finally {
        }
        */
    }
}
