package ai.shield.shieldaidemo;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


    }

    public void listenForAgents(View view) {
        boolean isSet = GamsNativeLibrary.getInstance().getStatus();
        TextView sampleText = ((TextView) findViewById(R.id.sample_text));
        if (isSet) {
            sampleText.setText(sampleText.getText() + "\nAll Agents ready...");
        } else {
            sampleText.setText(sampleText.getText() + "\nAgents not ready yet.");
        }
    }

    public void markAgentsReady(View view) {
        GamsNativeLibrary.getInstance().setReady();

    }


}
