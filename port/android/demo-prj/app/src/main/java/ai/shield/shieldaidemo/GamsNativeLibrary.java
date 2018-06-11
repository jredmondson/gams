package ai.shield.shieldaidemo;

import com.madara.KnowledgeBase;

/**
 * Created by Amit S on 21/05/18.
 * Followed the code from here - https://sourceforge.net/p/madara/wiki/JavaMadaraArchitecture/
 */
public class GamsNativeLibrary {
    private KnowledgeBase knowledge = new KnowledgeBase();

    static {
//        System.loadLibrary("boost_system");
//        System.loadLibrary("boost_atomic");
//        System.loadLibrary("boost_chrono");
//        System.loadLibrary("boost_container");
//        System.loadLibrary("boost_context");
//        System.loadLibrary("boost_contract");
//        System.loadLibrary("boost_coroutine");
//        System.loadLibrary("boost_date_time");
//        System.loadLibrary("boost_filesystem");
//        System.loadLibrary("boost_graph");
//        System.loadLibrary("boost_iostreams");
//        System.loadLibrary("boost_log");
//        System.loadLibrary("boost_log_setup");
//        System.loadLibrary("boost_math_c99");
//        System.loadLibrary("boost_math_c99f");
//        System.loadLibrary("boost_math_c99l");
//        System.loadLibrary("boost_math_tr1");
//        System.loadLibrary("boost_math_tr1f");
//        System.loadLibrary("boost_math_tr1l");
//        System.loadLibrary("boost_prg_exec_monitor");
//        System.loadLibrary("boost_program_options");
//        System.loadLibrary("boost_random");
//        System.loadLibrary("boost_regex");
//        System.loadLibrary("boost_serialization");
//        System.loadLibrary("boost_signals");
//        System.loadLibrary("boost_stacktrace_basic");
//        System.loadLibrary("boost_stacktrace_noop");
//        System.loadLibrary("boost_system");
//        System.loadLibrary("boost_thread");
//        System.loadLibrary("boost_timer");
//        System.loadLibrary("boost_type_erasure");
//        System.loadLibrary("boost_unit_test_framework");
//        System.loadLibrary("boost_wave");
//        System.loadLibrary("boost_wserialization");

        
        System.loadLibrary("Madara_Jar");
        System.loadLibrary("MADARA");
        System.loadLibrary("gams_jar");
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
