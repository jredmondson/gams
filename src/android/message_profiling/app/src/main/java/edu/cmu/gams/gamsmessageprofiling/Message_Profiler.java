package edu.cmu.gams.gamsmessageprofiling;

/**
 * Created by aldukeman on 6/16/15.
 */
public class Message_Profiler {
    enum transport_t
    {
        MULTICAST,
        BROADCAST,
        UDP
    }

    transport_t type;
    String address;
    int size;
    int rate;
    int duration;
    int id;

    public void run ()
    {

    }

    public Message_Profiler (String t, String a, int s, int r, int d, int i)
    {
        switch(t)
        {
            case "Multicast":
                type = transport_t.MULTICAST;
                break;
            case "Broadcast":
                type = transport_t.BROADCAST;
                break;
            case "UDP":
                type = transport_t.UDP;
                break;
        }

        address = a;
        size = s;
        rate = r;
        duration = d;
        id = i;
    }
}
