package com.disnodeteam.dogecv;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.view.Surface;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.Mat;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by guinea on 6/19/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a base class for an OpenCV pipeline loop. In most cases, one would want to override processFrame() with their own function.
 */

public abstract class OpenCVPipeline_Modified implements CameraBridgeViewBase.CvCameraViewListener2 {

    //Loads the OpenCV library
    static {
        try {
            System.loadLibrary("opencv_java3");
        } catch (UnsatisfiedLinkError e) {
            OpenCVLoader.loadOpenCV();
            // pass
        }
    }

    //Configurables. You probably don't need to change these.
    public static final int backCameraID = 0;
    public static final int frontCameraID = 1;


    //OpenCV-related
    protected JavaCameraView cameraView;
    private volatile ViewDisplay viewDisplay;
    protected Context context;
    private boolean initStarted = false;
    private boolean inited = false;

    //DogeCV logic
    private DogeCV.CameraMode cameraMode = DogeCV.CameraMode.BACK;
    private volatile int cameraIndex = 0;
    public  int degrees = 0;


    /**
     * Initializes the OpenCVPipeline, but implicitly uses the rear camera, without Vuforia or VuMarks
     * @param context the application context, usually hardwareMap.appContext
     * @param viewDisplay the ViewDisplay that will display the underlying JavaCameraView to the screen;
     *                    in most cases, using CameraViewDisplay.getInstance() as the argument is just fine.
     */
    public void init(Context context, ViewDisplay viewDisplay, DogeCV.CameraMode cameraMode) {
        init(context, viewDisplay, cameraMode, false, null);
    }


    /**
     * Initializes the OpenCVPipeline.
     * @param context the application context, usually hardwareMap.appContext
     * @param viewDisplay the ViewDisplay that will display the underlying JavaCameraView to the screen;
     *                    in most cases, using CameraViewDisplay.getInstance() as the argument is just fine.
     * @param cameraMode Which camera is to be used, will be a DogeCV.CameraMode
     * @param findVuMarks A boolean. True to scan for vumarks, false to ignore them
     * @param webcamName The CameraName representing the webcam to be used
     */
    public void init(Context context, ViewDisplay viewDisplay, DogeCV.CameraMode cameraMode, boolean findVuMarks, CameraName webcamName) {
        this.initStarted = true;
        this.viewDisplay = viewDisplay;
        this.context = context;
        this.cameraMode = cameraMode;
        //Sets up camera
        switch (this.cameraMode) {
            case BACK:
                this.cameraIndex = backCameraID;
                break;
            case FRONT:
                this.cameraIndex = frontCameraID;
                break;
        }


        //Starts CV on separate thread
        final Activity activity = (Activity) context;
        final Context finalContext = context;
        final CameraBridgeViewBase.CvCameraViewListener2 self = this;
        //final int cameraMoniterViewID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JavaCameraViews must be instantiated on a UI thread
                cameraView = new CustomCameraView(finalContext, getCameraIndex());
                cameraView.setCameraIndex(getCameraIndex());
                cameraView.setCvCameraViewListener(self);
                cameraView.enableFpsMeter();
                inited = true;
            }
        });
    }

    /**
     * Enables the detector.
     * This function must be called AFTER init().
     * @throws IllegalStateException if enable() is called before init()
     */
    public void enable() {
        if (!initStarted) throw new IllegalStateException("init() needs to be called before an OpenCVPipeline can be enabled!");
        // this is an absolute hack
        try {
            while (!inited) Thread.sleep(10);
        } catch (InterruptedException e) { return; }
        //Runs enabling sequence for Dogeforia
        cameraView.enableView();
        viewDisplay.setCurrentView(context, getCameraView());
    }

    /**
     * Detaches the JavaCameraView from the camera and the screen, stopping OpenCV processing.
     * Be careful not to:
     *     - disable twice
     *     - disable before enabling
     * because dean kamen help you if something bad happens from that
     */
    public void disable() {
        cameraView.disableView();
        viewDisplay.removeCurrentView(context);
    }

    /**
     * Exposes the underlying JavaCameraView used. Before init() is called, this is null.
     * @return the JavaCameraView.
     */
    public JavaCameraView getCameraView() {
        return cameraView;
    }


    /**
     * Exposes the index of the camera used
     * @return
     */
    protected synchronized int getCameraIndex() {return cameraIndex;}



    /**
     * This function is called when the camera is started; overriding this may be useful to set the
     * maximum width and height parameters of an image processing pipeline.
     * @param width -  the width of the frames that will be delivered
     * @param height - the height of the frames that will be delivered
     */
    @Override
    public void onCameraViewStarted(int width, int height) {}

    /**
     * Override this function if there should be logic on camera close.
     */
    @Override
    public void onCameraViewStopped() {}

    /**
     * The method that calls {@link #processFrame(Mat, Mat)}; there's little reason to override this, if ever.
     * @param inputFrame the input frame given by the internal JavaCameraView
     * @return the result of {@link #processFrame(Mat, Mat)}
     */
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgba = new Mat();
        Mat gray = new Mat();
        rgba = inputFrame.rgba();
        gray = inputFrame.gray();

        switch (((Activity) context).getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_0:
                if(cameraIndex == 0){ //Need to rotate differently depending on front or back camera
                    Core.rotate(rgba, rgba, Core.ROTATE_90_CLOCKWISE);
                    Core.rotate(gray, gray, Core.ROTATE_90_CLOCKWISE);
                } else if(cameraIndex == 1){
                    Core.rotate(rgba, rgba, Core.ROTATE_90_COUNTERCLOCKWISE);
                    Core.rotate(gray, gray, Core.ROTATE_90_COUNTERCLOCKWISE);
                }
                break;
            case Surface.ROTATION_90:
                rgba = inputFrame.rgba();
                gray = inputFrame.gray();
                break;
            case Surface.ROTATION_180:
                if(cameraIndex == 0){ //Need to rotate differently depending on front or back camera
                    Core.rotate(rgba, rgba, Core.ROTATE_90_COUNTERCLOCKWISE);
                    Core.rotate(gray, gray, Core.ROTATE_90_COUNTERCLOCKWISE);
                } else if(cameraIndex == 1){
                    Core.rotate(rgba, rgba, Core.ROTATE_90_CLOCKWISE);
                    Core.rotate(gray, gray, Core.ROTATE_90_CLOCKWISE);
                }
                break;
            case Surface.ROTATION_270:
                Core.rotate(rgba, rgba, Core.ROTATE_180);
                Core.rotate(gray, gray, Core.ROTATE_180);
                break;
        }
        return processFrame(rgba, gray);
    }

    /**
     * Override this with the main image processing logic. This is run every time the camera receives a frame.
     * @param rgba a {@link Mat} that is in RGBA format
     * @param gray a {@link Mat} that is already grayscale
     * @return the Mat that should be displayed to the screen; in most cases one would probably just want to return rgba
     */
    public abstract Mat processFrame(Mat rgba, Mat gray);

}