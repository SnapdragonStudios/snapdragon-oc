//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

package com.qualcomm.sdocdemo;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageView;
import android.os.Environment;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;


public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    Boolean sampleCapExist = false;
    String appExternalDirPath="";
    private void ExtractResource()
    {
        File appExternalDir = getApplicationContext().getExternalFilesDir("");
        appExternalDirPath = appExternalDir.getAbsolutePath();
        String expectInputCapPath = appExternalDirPath+ "/input.cap";
        File sample = new File(expectInputCapPath);
        sampleCapExist = sample.exists();
        if(sampleCapExist == false )
        {
            Environment.getExternalStorageDirectory().toString();
            try {
                String[] assetItems = getResources().getAssets().list("");
                for(int idx = 0; idx<assetItems.length; idx++)
                {
                    if(assetItems[idx].contains(".cap"))
                    {
                        AssetManager assetFiles = getAssets();

                        InputStream in = assetFiles.open(assetItems[idx]);
                        OutputStream out = new FileOutputStream(expectInputCapPath);

                        copyAssetFiles(in, out);
                        in.close();
                        out.flush();
                        out.close();
                        System.out.println(" assetItems:" + assetItems[idx]);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }

            sampleCapExist = sample.exists();
        }
    }

    private static void copyAssetFiles(InputStream in, OutputStream out) {
        try {
            byte[] buffer = new byte[1024];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    int width = 1152;
    int height = 384;
    public int[] readPGMP5Data(String pgmFile)
    {
        try {
            FileInputStream fin = new FileInputStream(pgmFile);
            DataInputStream inputStream  = new DataInputStream(fin);
            String format = inputStream.readLine();
            String size = inputStream.readLine();
            String line = inputStream.readLine();
            //analyze width/height here

            int heightIdx = size.indexOf(" ");
            if(heightIdx >= 0){
                String heightStr = size.substring(heightIdx+1).replace(" ", "");
                height = Integer.parseInt(heightStr);
                width = Integer.parseInt(size.substring(0, heightIdx));
            }

            System.out.println(" width:" + width + " Height " + height);

            if(format.equals( "P5") && line.equals(("255"))  )
            {
                int[] pixels = new int[width * height];
                for (int i = 0; i < width * height; i++)
                {
                    int b = (char) inputStream.readUnsignedByte();
                    pixels[i] =  b;
                }
                return pixels;
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }catch(IOException e1){System.out.println("read exception");}
        return null;
    }


    private Boolean requestStoragePermission() {
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.READ_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, android.Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED)
            return true;
        ActivityCompat.requestPermissions(this, new String[]
                {
                        android.Manifest.permission.READ_EXTERNAL_STORAGE,
                        android.Manifest.permission.WRITE_EXTERNAL_STORAGE,
                }, PERMISSION_REQUEST_CODE);
        return false;
    }
    private static final int PERMISSION_REQUEST_CODE = 200;
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        //Checking the request code of our request
        if (requestCode == PERMISSION_REQUEST_CODE) {
            //If permission is granted
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                //Displaying a toast
            } else {
                //Toast.makeText(this, "Access to sdcard is needed. Please grant the permission", Toast.LENGTH_LONG).show();
                System.exit(0);
            }
        }
        StartSDOC();
    }

    ImageView imageView;
    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        imageView= (ImageView) findViewById(R.id.imageView);
        imageView.setVisibility(View.INVISIBLE);

        this.setTitle("SDOC " + GetPackageArmVersion());
        if(requestStoragePermission())
        {
            StartSDOC();
        }
    }
    static Boolean sdocCalculated = false;
    static Boolean sdocStressTestStarted = false;

    int stressTestRound = 0;
    void StartSDOC(){
        ExtractResource();

        if(sdocCalculated == false)
        {
            sdocCalculated = true;
            String jni = CallSDOCInJNIQuick();
            System.out.println(jni);
        }

        String extStorageDirectory = Environment.getExternalStorageDirectory().toString();
        File capPath = new File(appExternalDirPath + "/all/");
        if (!capPath.exists() || !capPath.isDirectory()) {
            String targetPgMFile = appExternalDirPath+ "/depthbuffer.pgm";
            int[] bytes = readPGMP5Data(targetPgMFile);
            if(bytes != null)
            {
                Bitmap bitmap = Bitmap.createBitmap(height,width, Bitmap.Config.ARGB_8888);
                int[] bytesRotate = new int[width * height];
                for(int x = 0; x < height; x++){
                    for(int y=0; y<width; y++){
                        int byteIdx = (height-1-x) + y * height; //current idx
                        int originalIdx = x * width + y;
                        int value = bytes[originalIdx];
                        bytesRotate[byteIdx] = (255 << 24) | (value << 16) | (value << 8) | value;
                    }
                }
                bitmap.setPixels(bytesRotate, 0, height, 0, 0, height, width);

                imageView.setImageBitmap(bitmap);
                imageView.setVisibility(View.VISIBLE);


                if(sdocStressTestStarted == false)
                {
                    sdocStressTestStarted = true;
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            System.out.println(" A new thread to stress test SDOC ");
                            String jni = CallSDOCStressTestInJNI();
                            System.out.println(jni);
                        }
                    }).start();
                }

            }
            else{
                System.exit(0);
            }
        }
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String CallSDOCStressTestInJNI();
    public native String CallSDOCInJNIQuick();
    public native String GetPackageArmVersion();
}