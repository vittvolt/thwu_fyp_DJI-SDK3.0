package com.dji.fpvtutorial;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;

public class StarterActivity extends Activity {

    Button button_phone;
    Button button_pc;
    Button start_button;

    int selection = 1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_starter);

        button_phone = (Button) findViewById(R.id.phone_selection);
        button_pc = (Button) findViewById(R.id.pc_selection);
        start_button = (Button) findViewById(R.id.start_button);

        button_phone.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                button_phone.setPressed(true);
                selection = 1;

                button_pc.setPressed(false);
                return true;
            }
        });

        button_pc.setOnTouchListener(new View.OnTouchListener(){
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                button_pc.setPressed(true);
                selection = 2;

                button_phone.setPressed(false);
                return true;
            }
        });

        start_button.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                Intent home = new Intent(getApplicationContext(), uavTrackingMain.class);
                home.putExtra("selection", selection);
                startActivity(home);
                finish();
            }
        });
    }
}
