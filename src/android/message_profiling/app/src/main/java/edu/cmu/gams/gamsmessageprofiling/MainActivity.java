package edu.cmu.gams.gamsmessageprofiling;

import android.content.Context;
import android.net.DhcpInfo;
import android.net.wifi.WifiManager;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;

import java.io.IOException;
import java.net.InetAddress;

public class MainActivity extends ActionBarActivity
{
    private static final String LOCAL_TAG = "MainActivity";

    public void runButtonOnClick (View v) throws IOException
    {
        final Button run = (Button)findViewById(R.id.run_button);
        run.setClickable(false);
        Runtime rt = Runtime.getRuntime();
        long maxMemory = rt.maxMemory();
        Log.d(LOCAL_TAG, "Max Memory Available: " + Long.toString(maxMemory));
        Log.d(LOCAL_TAG, "MADARA_VERSION: " + ai.madara.util.Utility.getVersion());

        Spinner type_spinner = (Spinner) findViewById (R.id.type_spinner);
        String type = type_spinner.getSelectedItem().toString();

        String address_text = ((EditText) findViewById(R.id.address_tex_box)).getText().toString();
        String port_text = ((EditText)findViewById(R.id.port_text_box)).getText().toString();
        address_text += ":" + port_text;
        int size = Integer.parseInt(((EditText) findViewById(R.id.size_text_box)).getText().toString());
        int rate = Integer.parseInt(((EditText) findViewById(R.id.rate_text_box)).getText().toString());
        int dur = Integer.parseInt(((EditText) findViewById(R.id.duration_text_box)).getText().toString());
        int id = Integer.parseInt(((EditText) findViewById(R.id.id_text_box)).getText().toString());
        int swarmSize = Integer.parseInt(((EditText) findViewById(R.id.swarm_size_text_box)).getText().toString());

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

        ai.madara.logger.GlobalLogger.setLevel(0);
        Message_Profiler profiler = new Message_Profiler(type, address_text, size, rate, dur, id, swarmSize);
        profiler.start();
        try
        {
            profiler.join();
        }
        catch (Exception e)
        {
        }
        finally
        {
        }
        ai.madara.logger.GlobalLogger.setLevel(0);

        run.setClickable(true);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        ai.madara.logger.GlobalLogger.addTerm();

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Spinner type_spinner = (Spinner) findViewById (R.id.type_spinner);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
                R.array.transport_types, android.R.layout.simple_spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        type_spinner.setAdapter(adapter);

        EditText address_text = (EditText) findViewById(R.id.address_tex_box);
        address_text.setText("239.255.0.1");

        EditText port_text = (EditText) findViewById(R.id.port_text_box);
        port_text.setText("4150");

        EditText size_text = (EditText) findViewById(R.id.size_text_box);
        size_text.setText("10");

        EditText rate_text = (EditText) findViewById(R.id.rate_text_box);
        rate_text.setText("5");

        EditText duration_text = (EditText) findViewById(R.id.duration_text_box);
        duration_text.setText("10");

        EditText agent_id_text = (EditText) findViewById(R.id.id_text_box);
        agent_id_text.setText("0");

        EditText swarm_size_text = (EditText) findViewById(R.id.swarm_size_text_box);
        swarm_size_text.setText("2");
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
