package ai.shield.shieldaidemo;

import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;

import ai.madara.exceptions.MadaraDeadObjectException;
import ai.madara.knowledge.EvalSettings;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.knowledge.Variables;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.TransportContext;
import ai.madara.transport.TransportType;
import ai.madara.transport.filters.AggregateFilter;
import ai.madara.transport.filters.Packet;

/**
 * Created by Amit S on 08/07/18.
 */
public class MadaraNetworkTester extends AppCompatActivity {

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_network);
        getSupportActionBar().setTitle("Networking Test");

        sendMessage(null);



    }

    public void sendMessage(View view) {
        new NetworkThread().start();
    }


    private class NetworkThread extends Thread{
        @Override
        public void run() {
            try {
                QoSTransportSettings settings = new QoSTransportSettings();
                settings.setHosts(new String[]{"239.255.0.1:4150"});
                settings.setType(TransportType.MULTICAST_TRANSPORT);

                // add the above filter for all file types, applied before sending
                // settings.addSendFilter (new AddOrEraseId());
                settings.addReceiveFilter(new AggregateFilter() {
                    @Override
                    public void filter(Packet packet, TransportContext context, Variables variables) {
                        Log.i("MadaraNetworkTester", "Received Message: ");
                        String[] keys = new String[0];
                        try {
                            keys = packet.getKeys();
                        } catch (MadaraDeadObjectException e) {
                            e.printStackTrace();
                        }
                        int i = 0;
                        for (String key : keys) {
                            try {
                                Log.i("MadaraNetworkTester", "Received: " + key + " == " + packet.get(key).toString());
                            } catch (MadaraDeadObjectException e) {
                                e.printStackTrace();
                            }
                            i++;
                        }
                    }
                });

                // create a knowledge base with the multicast transport settings
                KnowledgeBase knowledge = new KnowledgeBase("", settings);
                EvalSettings queueUntilLater = new EvalSettings();
                queueUntilLater.setDelaySendingModifieds(true);

                // set id so we have access to it in the aggregate outgoing filter
                knowledge.set(".id", 1);

                // build a packet from this id with some information
                knowledge.set("occupation", "Banker", queueUntilLater);
                knowledge.set("age", 43, queueUntilLater);

                // a final piece of data with default settings will activate the
                // aggregate filter
                knowledge.set("money", 553200.50);


                knowledge.evaluate("amit_ready = 1");

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

}
