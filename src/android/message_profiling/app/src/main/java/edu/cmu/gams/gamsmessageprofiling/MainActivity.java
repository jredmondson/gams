package edu.cmu.gams.gamsmessageprofiling;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;


public class MainActivity extends ActionBarActivity {

    public void runButtonOnClick (View v)
    {
        Spinner type_spinner = (Spinner) findViewById (R.id.type_spinner);
        String type = type_spinner.getSelectedItem().toString();

        String address_text = ((EditText) findViewById(R.id.address_tex_box)).getText().toString();
        String port_text = ((EditText)findViewById(R.id.port_text_box)).getText().toString();
        address_text += ":" + port_text;
        int size = Integer.parseInt(((EditText) findViewById(R.id.size_text_box)).getText().toString());
        int rate = Integer.parseInt(((EditText) findViewById(R.id.rate_text_box)).getText().toString());
        int dur = Integer.parseInt(((EditText) findViewById(R.id.duration_text_box)).getText().toString());
        int id = Integer.parseInt(((EditText) findViewById(R.id.id_text_box)).getText().toString());

        Message_Profiler profiler = new Message_Profiler(type, address_text, size, rate, dur, id);
        profiler.run();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
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
        size_text.setText("1000");

        EditText rate_text = (EditText) findViewById(R.id.rate_text_box);
        rate_text.setText("10");

        EditText duration_text = (EditText) findViewById(R.id.duration_text_box);
        duration_text.setText("15");

        EditText agent_id_text = (EditText) findViewById(R.id.id_text_box);
        agent_id_text.setText("0");
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
