package edu.cmu.gams.gamscontroller;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import ai.gams.controllers.BaseController;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.knowledge.KnowledgeList;
import ai.madara.knowledge.KnowledgeRecord;
import ai.madara.transport.QoSTransportSettings;
import ai.madara.transport.TransportSettings;
import ai.madara.transport.TransportType;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

/**
 * Created by aldukeman on 7/20/15.
 */
public class ControllerWrapper extends Thread
{
    private static final String LOCAL_TAG = "ControllerWrapper";

    private TransportType transportType;
    private String transportAddress;
    private int id;
    private int swarmSize;
    private String algorithm;
    private String arg;
    private int duration;
    private int rate;
    private String vrepHost;
    private int vrepPort;
    private AssetManager assets;
    private File cacheDir;

    public ControllerWrapper (TransportType transportType, String transportAddress, int id,
                              int swarmSize, String algorithm, String arg, int duration, int rate,
                              String vrepHost, int vrepPort, AssetManager assets, File cacheDir)
    {
        this.transportType = transportType;
        this.transportAddress = transportAddress;
        this.id = id;
        this.swarmSize = swarmSize;
        this.algorithm = algorithm;
        this.arg = arg;
        this.duration = duration;
        this.rate = rate;
        this.vrepHost = vrepHost;
        this.vrepPort = vrepPort;
        this.assets = assets;
        this.cacheDir = cacheDir;
    }

    private String loadPlatformModelToCache(String platform) throws IOException
    {
        String file = "";
        switch(platform)
        {
            case "vrep-uav":
            case "vrep_uav":
            case "vrep-quad":
            case "vrep_quad":
                file = "Quadricopter_NoCamera.ttm";
                break;
        }

        File cacheFile = new File(cacheDir, file);
        try {
            InputStream inputStream = assets.open(file);
            try {
                FileOutputStream outputStream = new FileOutputStream(cacheFile);
                try {
                    byte[] buf = new byte[1024];
                    int len;
                    while ((len = inputStream.read(buf)) > 0) {
                        outputStream.write(buf, 0, len);
                    }
                } finally {
                    outputStream.close();
                }
            } finally {
                inputStream.close();
            }
        } catch (IOException e) {
            throw new IOException("Could not open " + file, e);
        }
        return cacheFile.getAbsolutePath();
    }

    private void loadMadaraFiles(KnowledgeBase knowledge)
    {
        Log.d(LOCAL_TAG, "reading asset files");
        BufferedReader in;
        String madaraInit;
        try
        {
            StringBuilder buf = new StringBuilder();
            InputStream input = assets.open("areas/small.mf");
            in = new BufferedReader(new InputStreamReader(input));
            String str;
            while ((str = in.readLine ()) != null)
            {
                buf.append (str);
                buf.append (System.getProperty("line.separator"));
            }
            input = assets.open("madara_init_common.mf");
            in = new BufferedReader(new InputStreamReader(input));
            while ((str = in.readLine()) != null)
            {
                buf.append (str);
                buf.append (System.getProperty("line.separator"));
            }
            madaraInit = buf.toString();
        }
        catch (IOException e)
        {
            Log.e(LOCAL_TAG, "Error loading areas/cmu.mf asset");
            return;
        }
        knowledge.evaluateNoReturn(madaraInit);
    }

    public void run ()
    {
        // load the correct model file into cache
        String modelFile;
        try
        {
            modelFile = loadPlatformModelToCache("vrep-uav");
            Log.d(LOCAL_TAG, "loaded model to " + modelFile);
        }
        catch (IOException e)
        {
            Log.d(LOCAL_TAG, "failed to load model to cache");
            return;
        }

        // setup knowledge base
        Log.d(LOCAL_TAG, "creating knowledge base");
        QoSTransportSettings settings = new QoSTransportSettings();
        settings.setHosts(new String[]{transportAddress});
        settings.setType(transportType);
        KnowledgeBase knowledge = new KnowledgeBase(Integer.toString(id), settings);

        // get madara init files as string for knowledge base evaluation
        loadMadaraFiles(knowledge);

        // set GAMS stuff
        Log.d(LOCAL_TAG, "Setting stuff in knowledge base");
        knowledge.set(".id", id);
        knowledge.set(".vrep_host", vrepHost);
        knowledge.set(".vrep_port", vrepPort);

        // Setup controller
        Log.d(LOCAL_TAG, "Setting up controller");
        BaseController controller = new BaseController(knowledge);
        controller.initVars(id, swarmSize);

        // init platform
        Log.d(LOCAL_TAG, "initializing platform");
        long[] records = new long[2];
        KnowledgeRecord file = new KnowledgeRecord(modelFile);
        records[0] = file.getCPtr();
        KnowledgeRecord clientSide = new KnowledgeRecord(1);
        records[1] = clientSide.getCPtr();
        KnowledgeList args = new KnowledgeList(records);
        controller.initPlatform("vrep-uav", args);
        Log.d(LOCAL_TAG, "done initializing platform");

        // init algorithm
        KnowledgeRecord a = new KnowledgeRecord(arg);
        records = new long[1];
        records[0] = a.getCPtr();
        args = new KnowledgeList(records);
        Log.d(LOCAL_TAG, "initializing algorithm");
        controller.initAlgorithm(algorithm, args);
        Log.d(LOCAL_TAG, "done initializing algorithm");

        // run the controller
        Log.d(LOCAL_TAG, "run the controller");
        controller.run(1.0 / rate, duration);
        Log.d(LOCAL_TAG, knowledge.toString());
    }
}
