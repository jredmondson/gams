package ai.shield.shieldaidemo;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Spinner;
import android.widget.TextView;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Method;

public class MadaraTestsRunner extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_madara_tests);

        ((Spinner) findViewById(R.id.tests)).setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
                String testName = adapterView.getSelectedItem().toString();
                performTest(testName);
            }

            @Override
            public void onNothingSelected(AdapterView<?> adapterView) {

            }
        });

    }

    private void performTest(String testName) {

        if ("NetworkTest".equalsIgnoreCase(testName)) {
            startActivity(new Intent(this, MadaraNetworkTester.class));
            return;
        }

        try {

            String findClass = "ai.madara.tests." + testName;
            Class clz = Class.forName(findClass);
            Object o = clz.newInstance();
            Method mainMethod = clz.getDeclaredMethod("main", String[].class);
            String[] params = null; // init params accordingly
            mainMethod.invoke(o, (Object) params);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    private StringBuilder readLogs() {
        StringBuilder logBuilder = new StringBuilder();
        try {
            Process process = Runtime.getRuntime().exec("logcat -d");
            BufferedReader bufferedReader = new BufferedReader(
                    new InputStreamReader(process.getInputStream()));

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                logBuilder.append(line + "\n");
            }
        } catch (IOException e) {
        }
        return logBuilder;
    }


    public void showLogs(View view) {
        StringBuilder builder = readLogs();
        ((TextView) findViewById(R.id.logcat)).setText(builder.toString());
    }


}
