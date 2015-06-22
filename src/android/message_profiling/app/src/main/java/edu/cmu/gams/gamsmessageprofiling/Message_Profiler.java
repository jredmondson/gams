package edu.cmu.gams.gamsmessageprofiling;

import android.util.Log;

import com.gams.controllers.BaseController;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeList;
import com.madara.KnowledgeRecord;
import com.madara.MadaraLog;
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

    public void run ()
    {
        TransportSettings settings = new TransportSettings();
        String[] hosts = new String[1];
        hosts[0] = address;
        Log.d(TAG, address);
        settings.setHosts(hosts);
        settings.setType(type);

        Log.d(TAG, "creating knowledge base...");
        String host = Integer.toString(id);
        KnowledgeBase knowledge = new KnowledgeBase(host, settings);
        //KnowledgeBase knowledge = new KnowledgeBase();
        Log.d(TAG, "done creating knowledge base");

        String prefix = "device." + id + ".command";

        BaseController controller = new BaseController(knowledge);
        controller.initVars(0, 1);
        controller.initPlatform("null", new KnowledgeList(new long[0]));
        knowledge.set(".id", id);
        knowledge.set(prefix, "message profiling");
        knowledge.set(prefix + ".size", 1);
        knowledge.set(prefix + ".0", 10);
        controller.run(1.0 / rate, duration);
    }

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
}
