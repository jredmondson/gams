package edu.cmu.gams.gamsmessageprofiling;

import android.content.Context;
import android.net.wifi.WifiManager;
import android.util.Log;
import android.os.Bundle;

import com.gams.algorithms.MessageProfiling;
import com.gams.controllers.BaseController;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeList;
import com.madara.KnowledgeRecord;
import com.madara.MadaraLog;
import com.madara.transport.QoSTransportSettings;
import com.madara.transport.TransportSettings;
import com.madara.transport.TransportType;

/**
 * Created by aldukeman on 6/16/15.
 */
class Message_Profiler {

    private TransportType type;
    private String address;
    private int size;
    private int rate;
    private int duration;
    private int id;

    private static final String TAG = "Message_Profiler";

    public Message_Profiler (String t, String a, int s, int r, int d, int i)
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
        size = s;
        rate = r;
        duration = d;
        id = i;
    }

    public void run ()
    {
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
        com.madara.logger.GlobalLogger.setLevel(6);
        controller.run(1.0 / rate, duration);
        com.madara.logger.GlobalLogger.setLevel(0);

        Log.d(TAG, knowledge.toString());

        /*try {
            com.madara.logger.GlobalLogger.setLevel(6);
            String[] args = new String[6];
            args[0] = "--type";
            if (id == 0)
                args[1] = "reader";
            else if (id == 1)
                args[1] = "writer";
            args[1] = "both";
            args[2] = "--duration";
            args[3] = "10";
            args[4] = "--host";
            args[5] = Integer.toString(id);
            com.gams.tests.TestMessagingThroughput.main (args);
        } catch (Exception e) {
            e.printStackTrace();
        }
        finally {
            com.madara.logger.GlobalLogger.setLevel(0);
        }*/
    }
}
