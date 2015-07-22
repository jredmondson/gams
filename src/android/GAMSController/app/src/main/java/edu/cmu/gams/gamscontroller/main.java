package edu.cmu.gams.gamscontroller;

import android.content.Context;
import android.net.DhcpInfo;
import android.net.wifi.WifiManager;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;

import com.madara.logger.GlobalLogger;
import com.madara.transport.TransportType;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ResourceBundle;


public class main extends ActionBarActivity {

    private static final String LOCAL_TAG = "GAMSController";

    public void runButtonOnClick (View v) throws IOException
    {
        final Button run = (Button)findViewById(R.id.run_button);
        run.setClickable(false);
        Runtime rt = Runtime.getRuntime();
        long maxMemory = rt.maxMemory();
        Log.d(LOCAL_TAG, "Max Memory Available: " + Long.toString(maxMemory));
        Log.d(LOCAL_TAG, "MADARA_VERSION: " + com.madara.util.Utility.getVersion());

        Spinner type_spinner = (Spinner) findViewById (R.id.transport_spinner);
        String type = type_spinner.getSelectedItem().toString();

        String address_text = ((EditText) findViewById(R.id.address_text)).getText().toString();
        String port_text = ((EditText)findViewById(R.id.port_text)).getText().toString();
        address_text += ":" + port_text;
        int id = Integer.parseInt(((EditText) findViewById(R.id.id_text)).getText().toString());
        int swarm_size = Integer.parseInt(((EditText) findViewById(R.id.swarm_size_text)).getText().toString());
        String algo = ((EditText) findViewById(R.id.algorithm_text)).getText().toString();
        String arg = ((EditText) findViewById(R.id.arg_text)).getText().toString();
        int dur = Integer.parseInt(((EditText) findViewById(R.id.duration_text)).getText().toString());
        int rate = Integer.parseInt(((EditText) findViewById(R.id.rate_text)).getText().toString());
        String vrep_address = ((EditText) findViewById(R.id.vrep_host_text)).getText().toString();
        int vrep_port = Integer.parseInt(((EditText) findViewById(R.id.vrep_port_text)).getText().toString());

        Log.d(LOCAL_TAG, "ID: " + id);

        if (type.equals ("Broadcast"))
        {
            WifiManager wifi = (WifiManager) getSystemService(Context.WIFI_SERVICE);
            DhcpInfo dhcp = wifi.getDhcpInfo();
            int broadcast = (dhcp.ipAddress & dhcp.netmask) | ~dhcp.netmask;
            byte[] quads = new byte[4];
            for (int k = 0; k < 4; ++k)
                quads[k] = (byte) ((broadcast >> k * 8) & 0xFF);
            address_text = InetAddress.getByAddress(quads).getHostAddress() + ":15000";
        }
        Log.d(LOCAL_TAG, "ADDRESS: " + address_text);

        com.madara.logger.GlobalLogger.setLevel(0);
        TransportType transportType = TransportType.MULTICAST_TRANSPORT;
        switch(type)
        {
            case "Multicast":
                transportType = TransportType.MULTICAST_TRANSPORT;
                break;
            case "Broadcast":
                transportType = TransportType.BROADCAST_TRANSPORT;
                break;
            case "UDP":
                transportType = TransportType.UDP_TRANSPORT;
                break;
        }

        Log.d(LOCAL_TAG, "creating wrapper");
        ControllerWrapper wrapper = new ControllerWrapper(transportType, address_text, id,
                swarm_size, algo, arg, dur, rate, vrep_address, vrep_port, getAssets(),
                getCacheDir());

        Log.d (LOCAL_TAG, "starting wrapper thread");
        com.gams.utility.Logging.setLevel(6);
        com.madara.logger.GlobalLogger.addTerm();
        com.madara.logger.GlobalLogger.setLevel(6);
        wrapper.start();
        try
        {
            Log.d(LOCAL_TAG, "waiting on wrapper to exit");
            wrapper.join();
        }
        catch (Exception e)
        {
        }
        finally
        {
        }
        com.madara.logger.GlobalLogger.setLevel(0);
        com.gams.utility.Logging.setLevel(0);

        run.setClickable(true);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        Spinner type_spinner = (Spinner) findViewById (R.id.transport_spinner);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.transport_types, android.R.layout.simple_spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        type_spinner.setAdapter(adapter);

        EditText address_text = (EditText) findViewById(R.id.address_text);
        address_text.setText("239.255.0.1");
        address_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText port_text = (EditText) findViewById(R.id.port_text);
        port_text.setText("4150");
        port_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText agent_id_text = (EditText) findViewById(R.id.id_text);
        agent_id_text.setText("0");
        agent_id_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText swarm_size_text = (EditText) findViewById(R.id.swarm_size_text);
        swarm_size_text.setText("1");
        swarm_size_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText algo_text = (EditText) findViewById(R.id.algorithm_text);
        algo_text.setText("urec");
        algo_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText arg_text = (EditText) findViewById(R.id.arg_text);
        arg_text.setText("region.0");
        arg_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText duration_text = (EditText) findViewById(R.id.duration_text);
        duration_text.setText("60");
        duration_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText rate_text = (EditText) findViewById(R.id.rate_text);
        rate_text.setText("5");
        rate_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText vrep_host_text = (EditText) findViewById(R.id.vrep_host_text);
        vrep_host_text.setText("172.20.10.4");
        vrep_host_text.setImeOptions(EditorInfo.IME_ACTION_DONE);

        EditText vrep_port_text = (EditText) findViewById(R.id.vrep_port_text);
        vrep_port_text.setText("19906");
        vrep_port_text.setImeOptions(EditorInfo.IME_ACTION_DONE);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
