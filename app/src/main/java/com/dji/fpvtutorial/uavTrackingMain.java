package com.dji.fpvtutorial;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.TextureView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.TextureView.SurfaceTextureListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import dji.sdk.AirLink.DJILBAirLink.DJIOnReceivedVideoCallback;
import dji.sdk.Camera.DJICamera;
import dji.sdk.Camera.DJICamera.CameraReceivedVideoDataCallback;
import dji.sdk.Codec.DJICodecManager;
import dji.sdk.FlightController.DJIFlightController;
import dji.sdk.FlightController.DJIFlightControllerDataType;
import dji.sdk.Products.DJIAircraft;
import dji.sdk.base.DJIBaseComponent.DJICompletionCallback;
import dji.sdk.base.DJIBaseProduct;
import dji.sdk.base.DJIBaseProduct.Model;
import dji.sdk.base.DJIError;
import dji.sdk.Camera.DJICameraSettingsDef.CameraMode;
import dji.sdk.Camera.DJICameraSettingsDef.CameraShootPhotoMode;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;


public class uavTrackingMain extends Activity implements View.OnTouchListener, SurfaceTextureListener,OnClickListener{

    //fyp report and poster
    String filename = "";
    int counter_file = 0;
    boolean collect_data = false;
    int counter_data = 1;

    //New variables
    float roll_control = 0;
    float throttle_control = 0;
    float pitch_control = 0;
    float yaw_control = 0;
    boolean in_operation = false;
    Handler handler_command = new Handler();

    boolean spin_clkwise = false;
    boolean spin_ctclkwise = false;
    boolean move_forward = false;
    boolean move_backward = false;

    DJIFlightController mController = null;
    boolean control = false;
    PID_Control mPID = new PID_Control();

    Button Enable, TakeOff, Landing, Spinning_CLKWise, Control, ScreenShot;
    TextView text1, text2;

    //Color blob detection variables
    private boolean              mIsColorSelected = false;
    private Mat mRgba;
    private Scalar mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;

    static{
        System.loadLibrary("opencv_java3");
    }
    int step_count = 0;

    private static final String TAG = uavTrackingMain.class.getName();

    private static final int INTERVAL_LOG = 300;
    private static long mLastTime = 0l;

    protected CameraReceivedVideoDataCallback mReceivedVideoDataCallBack = null;
    protected DJIOnReceivedVideoCallback mOnReceivedVideoCallback = null;

    private DJIBaseProduct mProduct = null;
    private DJICamera mCamera = null;
    // Codec for video live view
    protected DJICodecManager mCodecManager = null;

    protected TextView mConnectStatusTextView;
    //Video Preview
    protected TextureView mVideoSurface = null;
    TextureView mVideoSurface02 = null;
    private Button captureAction, recordAction, captureMode;
    private TextView viewTimer;
    private int i = 0;
    private int TIME = 1000;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fpvtutorial);

        initUI();

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataCallBack = new CameraReceivedVideoDataCallback() {

            @Override
            public void onResult(byte[] videoBuffer, int size) {
                if(mCodecManager != null){
                    // Send the raw H264 video data to codec manager for decoding
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }else {
                    Log.e(TAG, "mCodecManager is null");
                }
            }
        };

        // The callback for receiving the raw video data from Airlink
        mOnReceivedVideoCallback = new DJIOnReceivedVideoCallback() {

            @Override
            public void onResult(byte[] videoBuffer, int size) {
                if(mCodecManager != null){
                    // Send the raw H264 video data to codec manager for decoding
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        // Register the broadcast receiver for receiving the device connection's changes.
        IntentFilter filter = new IntentFilter();
        filter.addAction(djiBgApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        //Start the thread for sending the command at a constant frequency
        //test
        new Thread() {
            public void run() {
                if (mDetector==null)
                    handler_command.postDelayed(this,50);
                else if (control){
                    mPID.update_control_parameter(mDetector.x_err,mDetector.y_err,mDetector.z_err1);
                    pitch_control = (float) mPID.pitch_control;
                    roll_control = (float) mPID.roll_control;
                    throttle_control = (float) mPID.throttle_control;

                    if (Math.abs(pitch_control) < 0.1) pitch_control = 0;
                    if (Math.abs(pitch_control) > 2) pitch_control = 1.5f;
                    if (Math.abs(roll_control) < 0.1) roll_control = 0;
                    if (Math.abs(roll_control) > 2) roll_control = 1.5f;
                    if (Math.abs(throttle_control) < 0.1) throttle_control = 0;
                    if (Math.abs(throttle_control) > 0.9) throttle_control = 0.9f;
                    Send_Flight_Control_Command(pitch_control, roll_control, yaw_control, throttle_control);
                    handler_command.postDelayed(this,40);
                }
                else{
                    handler_command.postDelayed(this,40);
                }
            }
        }.start();
    }


    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        updateTitleBar();
        if(mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        uninitPreviewer();

        unregisterReceiver(mReceiver);

        super.onDestroy();
    }

    private void initUI() {

        //New buttons initialization
        text1 = (TextView) findViewById(R.id.text_display01);
        text2 = (TextView) findViewById(R.id.text_display02);

        Enable = (Button) findViewById(R.id.enable_control);
        TakeOff = (Button) findViewById(R.id.take_off);
        Landing = (Button) findViewById(R.id.landing);
        Control = (Button) findViewById(R.id.control);
        Spinning_CLKWise = (Button) findViewById(R.id.spin_clkwise);
        ScreenShot = (Button) findViewById(R.id.screen_shot);

        ScreenShot.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                new Thread() {
                    public void run() {
                        collect_data = !collect_data;
                        if (collect_data)
                            showToast("Data collecting began!");
                        else
                            showToast("Data collecting terminated!");
                        /*filename = "frame" + String.valueOf(counter_file)+".jpg";
                        counter_file++;
                        FileOutputStream out = null;
                        File sd = new File(Environment.getExternalStorageDirectory() + "/fyp_scr_sht");
                        if (!sd.exists()) {
                            sd.mkdirs();
                        }
                        File dest = new File(sd, filename);
                        try {
                            out = new FileOutputStream(dest);
                            Bitmap bmp = Bitmap.createBitmap(mRgba.cols(), mRgba.rows(), Bitmap.Config.ARGB_8888);
                            Utils.matToBitmap(mRgba, bmp);
                            bmp.compress(Bitmap.CompressFormat.JPEG, 80, out);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        finally {
                            try {
                                if (out != null) {
                                    out.close();
                                    showToast("OK!");
                                }
                            } catch (IOException e) {
                                Log.d(TAG, e.getMessage() + "Error");
                                e.printStackTrace();
                            }
                        } */
                    }
                }.start();
            }
        });

        Spinning_CLKWise.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    spin_clkwise = true;
                    new Thread() {
                        public void run() {
                            Spinning_CLKWise();
                            //Debug
                            /*runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    text1.setText(String.valueOf(k));
                                }
                            });
                            k = 1 - k; */
                            if(spin_clkwise && control){
                                handlerTimer.postDelayed(this,50);
                            }
                        }
                    }.start();
                }
                else if (event.getAction() == MotionEvent.ACTION_UP) {
                    spin_clkwise = false;
                }
                return false;
            }
        });

        mConnectStatusTextView = (TextView) findViewById(R.id.ConnectStatusTextView);
        // init mVideoSurface
        mVideoSurface = (TextureView)findViewById(R.id.video_previewer_surface);
        mVideoSurface02 = (TextureView) findViewById(R.id.video_previewer_surface02);
        mVideoSurface02.setOnTouchListener(this);

        viewTimer = (TextView) findViewById(R.id.timer);
        captureAction = (Button) findViewById(R.id.button1);
        recordAction = (Button) findViewById(R.id.button2);
        captureMode = (Button) findViewById(R.id.button3);

        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }

        captureAction.setOnClickListener(this);
        recordAction.setOnClickListener(this);
        captureMode.setOnClickListener(this);

        Enable.setOnClickListener(this);
        TakeOff.setOnClickListener(this);
        Landing.setOnClickListener(this);
        Control.setOnClickListener(this);
    }

    private Handler handlerTimer = new Handler();
    Runnable runnable = new Runnable(){
        @Override
        public void run() {
            // handler自带方法实现定时器
            try {

                handlerTimer.postDelayed(this, TIME);
                viewTimer.setText(Integer.toString(i++));

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    private void initPreviewer() {
        try {
            mProduct = djiBgApplication.getProductInstance();
        } catch (Exception exception) {
            mProduct = null;
        }

        if (null == mProduct || !mProduct.isConnected()) {
            mCamera = null;
            showToast(getString(R.string.disconnected));
        } else {


            if (null != mVideoSurface) {
                mVideoSurface.setSurfaceTextureListener(this);
            }

            if (!mProduct.getModel().equals(Model.UnknownAircraft)) {
                mCamera = mProduct.getCamera();
                if (mCamera != null){
                    // Set the callback
                    mCamera.setDJICameraReceivedVideoDataCallback(mReceivedVideoDataCallBack);

                }
            } else {
                if (null != mProduct.getAirLink()) {
                    if (null != mProduct.getAirLink().getLBAirLink()) {
                        // Set the callback
                        mProduct.getAirLink().getLBAirLink().setDJIOnReceivedVideoCallback(mOnReceivedVideoCallback);
                    }
                }
            }
        }
    }

    private void uninitPreviewer() {
        try {
            mProduct = djiBgApplication.getProductInstance();
        } catch (Exception exception) {
            mProduct = null;
        }

        if (null == mProduct || !mProduct.isConnected()) {
            mCamera = null;
            showToast(getString(R.string.disconnected));
        } else {
            if (!mProduct.getModel().equals(Model.UnknownAircraft)) {
                mCamera = mProduct.getCamera();
                if (mCamera != null){
                    // Set the callback
                    mCamera.setDJICameraReceivedVideoDataCallback(null);

                }
            } else {
                if (null != mProduct.getAirLink()) {
                    if (null != mProduct.getAirLink().getLBAirLink()) {
                        // Set the callback
                        mProduct.getAirLink().getLBAirLink().setDJIOnReceivedVideoCallback(null);
                    }
                }
            }
        }
    }

    //
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG,"onSurfaceTextureAvailable");
        if (mCodecManager == null) {
            Log.e(TAG, "mCodecManager is null 2");
            mCodecManager = new DJICodecManager(this, surface, width, height);
        }

        mRgba = new Mat();
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);
    }

    //
    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    //
    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        Log.e(TAG, "onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    //
    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        //String s1 = String.valueOf(mDetector.x + mDetector.w / 2) + "Y:" + String.valueOf(mDetector.y + mDetector.h / 2);
        String s1 = "pitch:" + String.valueOf(pitch_control) + "z_crt:" + String.valueOf(mDetector.z_err1);
        text1.setText(s1);
        String s2 = "z_int:" + String.valueOf(mPID.z_inte_error);
        text2.setText(s2);
        //text1.setText(String.valueOf(mDetector.w * mDetector.h));
        //String ss = "FrameSize: " + String.valueOf(mDetector.width) + "*" + String.valueOf(mDetector.height);
        //text2.setText(ss);

        if (step_count != 6){
            step_count++;
        }
        else{
            step_count = 0;

            new Thread(new Runnable() {
                @Override
                public void run() {
                    Bitmap frame_bmp = mVideoSurface.getBitmap();
                    Utils.bitmapToMat(frame_bmp, mRgba);  // frame_bmp is in ARGB format, mRgba is in RBGA format

                    //Todo: Do image processing stuff here
                    mRgba.convertTo(mRgba,-1,1.5,0);  // Increase intensity(light compensation) by 2

                    if (mIsColorSelected) {
                        //Show the error-corrected color
                        mBlobColorHsv = mDetector.get_new_hsvColor();
                        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

                        //Debug
                        Log.i(TAG, "mDetector rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");
                        Log.i(TAG, "mDetector hsv color: (" + mBlobColorHsv.val[0] + ", " + mBlobColorHsv.val[1] +
                                ", " + mBlobColorHsv.val[2] + ", " + mBlobColorHsv.val[3] + ")");

                        mDetector.process(mRgba);
                        List<MatOfPoint> contours = mDetector.getContours();
                        Log.e(TAG, "Contours count: " + contours.size());
                        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR, 2);

                        Mat colorLabel = mRgba.submat(4, 68, 4, 68);
                        colorLabel.setTo(mBlobColorRgba);

                        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
                        mSpectrum.copyTo(spectrumLabel);
                    }


                    Utils.matToBitmap(mRgba, frame_bmp);
                    Canvas canvas = mVideoSurface02.lockCanvas();
                    canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);
                    canvas.drawBitmap(frame_bmp, new Rect(0, 0, frame_bmp.getWidth(), frame_bmp.getHeight()),
                            new Rect((canvas.getWidth() - frame_bmp.getWidth()) / 2,
                                    (canvas.getHeight() - frame_bmp.getHeight()) / 2,
                                    (canvas.getWidth() - frame_bmp.getWidth()) / 2 + frame_bmp.getWidth(),
                                    (canvas.getHeight() - frame_bmp.getHeight()) / 2 + frame_bmp.getHeight()), null);
                    mVideoSurface02.unlockCanvasAndPost(canvas);

                    //FYP Report !!!!!!!!!!!!!!!!
                    if (collect_data){
                        String s = String.valueOf(counter_data) + " " + String.valueOf(mDetector.x+mDetector.w/2) + " " + String.valueOf(mDetector.y+mDetector.h/2);
                        s = s + " " + String.valueOf(mDetector.w * mDetector.h) + " " + String.valueOf(mDetector.width) + " " + String.valueOf(mDetector.height) + "\n";
                        generateNoteOnSD("data01.txt",s);
                        counter_data++;
                    }
                }
            }).start();
        }
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            updateTitleBar();
            onProductChange();
        }

    };


    private void updateTitleBar() {
        if(mConnectStatusTextView == null) return;
        boolean ret = false;
        DJIBaseProduct product = djiBgApplication.getProductInstance();
        if (product != null) {

            if(product.isConnected()) {
                //The product is connected
                mConnectStatusTextView.setText(djiBgApplication.getProductInstance().getModel() + " Connected");
                ret = true;
            } else {

                if(product instanceof DJIAircraft) {
                    DJIAircraft aircraft = (DJIAircraft)product;
                    if(aircraft.getRemoteController() != null && aircraft.getRemoteController().isConnected()) {
                        // The product is not connected, but the remote controller is connected
                        mConnectStatusTextView.setText("only RC Connected");
                        ret = true;
                    }
                }
            }
        }

        if(!ret) {
            // The product or the remote controller are not connected.
            mConnectStatusTextView.setText("Disconnected");
        }
    }

    protected void onProductChange() {
        initPreviewer();

    }

    @Override
    public boolean dispatchTouchEvent(MotionEvent ev) {
        if (ev.getAction() == MotionEvent.ACTION_DOWN) {
            final long current = System.currentTimeMillis();
            if (current - mLastTime < INTERVAL_LOG) {
                Log.d("", "click double");
                mLastTime = 0;
            } else {
                mLastTime = current;
                Log.d("", "click single");
            }
        }
        return super.dispatchTouchEvent(ev);
    }

    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(uavTrackingMain.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }



    @Override
    public void onClick(View v) {

        try {
            mProduct = djiBgApplication.getProductInstance();
        } catch (Exception exception) {
            mProduct = null;
        }

        if (null == mProduct || !mProduct.isConnected()) {
            mCamera = null;
            showToast(getString(R.string.disconnected));
            return;
        }

        switch (v.getId()) {
            case R.id.button1:{
                captureAction();
                break;
            }
            case R.id.button2:{
                recordAction();
                break;
            }
            case R.id.button3:{
                stopRecord();
                break;
            }
            case R.id.enable_control:{
                Enable_Virtual_Control();
                break;
            }
            case R.id.take_off:{
                Take_Off();
                break;
            }
            case R.id.landing:{
                Landing();
                break;
            }
            case R.id.control:{
                control = !control;
                if(!control){
                    showToast("Virtual control is turned off!");
                }
                else{
                    showToast("Virtual control is turned on!");
                }
            }
            default:
                break;
        }
    }
    // function for taking photo
    private void captureAction(){

        CameraMode cameraMode = CameraMode.ShootPhoto;

        mCamera = mProduct.getCamera();

        mCamera.setCameraMode(cameraMode, new DJICompletionCallback() {

            @Override
            public void onResult(DJIError error) {

                if (error == null) {
                    CameraShootPhotoMode photoMode = CameraShootPhotoMode.Single; // Set the camera capture mode as Single mode

                    mCamera.startShootPhoto(photoMode, new DJICompletionCallback() {

                        @Override
                        public void onResult(DJIError error) {
                            if (error == null) {
                                showToast("take photo: success");
                            } else {
                                showToast(error.getDescription());
                            }
                        }

                    }); // Execute the startShootPhoto API
                } else {
                    showToast(error.getDescription());
                }

            }

        });

    }
    // function for starting recording
    private void recordAction(){
        CameraMode cameraMode = CameraMode.RecordVideo;

        mCamera = mProduct.getCamera();

        mCamera.setCameraMode(cameraMode, new DJICompletionCallback() {

            @Override
            public void onResult(DJIError error) {

                if (error == null) {


                    mCamera.startRecordVideo(new DJICompletionCallback() {

                        @Override
                        public void onResult(DJIError error) {
                            if (error == null) {
                                showToast("Record video: success");
                                handlerTimer.postDelayed(runnable, TIME); // Start the timer for recording
                            } else {
                                showToast(error.getDescription());
                            }
                        }

                    }); // Execute the startShootPhoto API
                } else {
                    showToast(error.getDescription());
                }

            }

        });

    }
    // function for stopping recording
    private void stopRecord(){

        mCamera = mProduct.getCamera();

        mCamera.stopRecordVideo(new DJICompletionCallback() {

            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    showToast("Stop recording: success");
                } else {
                    showToast(error.getDescription());
                }
                handlerTimer.removeCallbacks(runnable); // Start the timer for recording
                i = 0; // Reset the timer for recording
            }

        });
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

    public boolean onTouch(View v, MotionEvent event) {
        //Todo: to give a notification for debugging
        //Very important! So that the onFrame function won't change the parameters before this section finishes
        mIsColorSelected = false;

        int rows = mRgba.rows();        int cols = mRgba.cols();


        int xOffset = (mVideoSurface02.getWidth() - cols) / 2;
        int yOffset = (mVideoSurface02.getHeight() - rows) / 2;

        int x = (int)event.getX() - xOffset;
        int y = (int)event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        org.opencv.core.Rect touchedRect = new org.opencv.core.Rect();

        touchedRect.x = (x>4) ? x-4 : 0;
        touchedRect.y = (y>4) ? y-4 : 0;

        touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width*touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        mDetector.resetStart();
        mDetector.setHsvColor(mBlobColorHsv);

        Log.i(TAG, "mDetector hsv color set: (" + mBlobColorHsv.val[0] + ", " + mBlobColorHsv.val[1] +
                ", " + mBlobColorHsv.val[2] + ", " + mBlobColorHsv.val[3] + ")");

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false;
    }

    public void Enable_Virtual_Control(){
        mController = ((DJIAircraft) mProduct).getFlightController();
        mController.enableVirtualStickControlMode(new DJICompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError == null){
                    showToast("Enable virtual controll success!");
                }
                else{
                    showToast(djiError.getDescription());
                }
            }
        });
    }

    public void Take_Off(){
        if(mController != null){
            mController.takeOff(new DJICompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null){
                        showToast("Take off success!");
                    }
                    else{
                        showToast(djiError.getDescription());
                    }
                }
            });
        }
    }

    public void Landing(){
        if (mController != null){
            mController.autoLanding(new DJICompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        showToast("Landing success!");
                    } else {
                        showToast(djiError.getDescription());
                    }
                }
            });
        }
    }

    public void Spinning_CLKWise(){
        mController.setHorizontalCoordinateSystem(DJIFlightControllerDataType.DJIVirtualStickFlightCoordinateSystem.Body);
        mController.setRollPitchControlMode(DJIFlightControllerDataType.DJIVirtualStickRollPitchControlMode.Angle);
        mController.setVerticalControlMode(DJIFlightControllerDataType.DJIVirtualStickVerticalControlMode.Velocity);
        mController.setYawControlMode(DJIFlightControllerDataType.DJIVirtualStickYawControlMode.AngularVelocity);

        mController.sendVirtualStickFlightControlData(new DJIFlightControllerDataType.DJIVirtualStickFlightControlData(0f, 0f, 40f, 0f), new DJICompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError != null) {
                    showToast(djiError.getDescription());
                }
            }
        });
    }

    public void Send_Flight_Control_Command(float pitch, float roll, float yaw, float throttle){
        mController.setHorizontalCoordinateSystem(DJIFlightControllerDataType.DJIVirtualStickFlightCoordinateSystem.Body);
        mController.setRollPitchControlMode(DJIFlightControllerDataType.DJIVirtualStickRollPitchControlMode.Angle);
        mController.setVerticalControlMode(DJIFlightControllerDataType.DJIVirtualStickVerticalControlMode.Velocity);
        mController.setYawControlMode(DJIFlightControllerDataType.DJIVirtualStickYawControlMode.AngularVelocity);

        mController.sendVirtualStickFlightControlData(new DJIFlightControllerDataType.DJIVirtualStickFlightControlData(pitch, roll, yaw, throttle), new DJICompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError != null) {
                    showToast(djiError.getDescription());
                }
            }
        });
    }

    // Write to file
    public void generateNoteOnSD(String sFileName, String sBody){
        try
        {
            File root = new File(Environment.getExternalStorageDirectory(), "fyp_tracking_data");
            if (!root.exists()) {
                root.mkdirs();
            }
            File gpxfile = new File(root, sFileName);
            FileWriter writer = new FileWriter(gpxfile, true);
            writer.append(sBody);
            writer.flush();
            writer.close();
        } catch(IOException e)
        {
            e.printStackTrace();
        }
    }
}
