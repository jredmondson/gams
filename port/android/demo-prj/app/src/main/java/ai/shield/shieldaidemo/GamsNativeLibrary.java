package ai.shield.shieldaidemo;

import ai.madara.knowledge.KnowledgeBase;

/**
 * Created by Amit S on 21/05/18.
 * Followed the code from here - https://sourceforge.net/p/madara/wiki/JavaMadaraArchitecture/
 */
public class GamsNativeLibrary {
    private KnowledgeBase knowledge = new KnowledgeBase();

    static {
       
        System.loadLibrary("MADARA_JNI");
        System.loadLibrary("MADARA");
        System.loadLibrary("GAMS_JNI");
        System.loadLibrary("GAMS");

    }

    private static GamsNativeLibrary instance;

    public static GamsNativeLibrary getInstance() {
        if (instance == null) {
            instance = new GamsNativeLibrary();
        }
        return instance;
    }

    public void setReady() {
        knowledge.set("all_agents_ready", 1.0);
    }

    public boolean getStatus() {
        return knowledge.get("all_agents_ready").toLong() == 1;
    }
}
