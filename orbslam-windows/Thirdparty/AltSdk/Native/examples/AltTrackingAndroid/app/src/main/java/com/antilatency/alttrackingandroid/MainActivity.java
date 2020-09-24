// Copyright (c) 2020 ALT LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of source code located below and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//  
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//  
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package com.antilatency.alttrackingandroid;

import android.os.Handler;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("AltTrackingExample");
        System.loadLibrary("AntilatencyDeviceNetwork");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView tv = findViewById(R.id.sample_text);
        tv.setMovementMethod(new ScrollingMovementMethod());

        Init();

        final Handler handler = new Handler();
        handler.post(new Runnable() {
            @Override
            public void run() {
                TextView tv = findViewById(R.id.sample_text);
                String text = tv.getText().toString() + GetOutput();

                String[] textArray = text.split("\n");
                String newText = "";
                int startIndex = 0;
                if(textArray.length > 200){
                    startIndex = textArray.length - 200;
                }

                for(int i = startIndex; i < textArray.length; ++i){
                    newText += textArray[i] + "\n";
                }

                tv.setText(newText);

                handler.postDelayed(this,500);
            }
        });
    }

    public native void Init();
    public native String GetOutput();
}
