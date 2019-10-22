package com.disnodeteam.dogecv.detectors;

import com.disnodeteam.dogecv.OpenCVPipeline_Modified;
import com.disnodeteam.dogecv.math.MathFTC;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;



/**
 * Created by Victo on 9/10/2018.
 * Updated October 2019 Codebusters 11471
 */

public abstract class DogeCVDetector_Modified extends OpenCVPipeline_Modified{

    public abstract Mat process(Mat input);

    private Size initSize;
    private Size adjustedSize;
    private Mat workingMat = new Mat();

    public Point cropTLCorner = null; //The top left corner of the image used for processing
    public Point cropBRCorner = null; //The bottom right corner of the image used for processing

    public double downscale = 0.5;
    public Size   downscaleResolution = new Size(640, 480);
    public boolean useFixedDownscale = true;
    protected String detectorName = "PuppyCV Detector";

    public DogeCVDetector_Modified(){
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        initSize = rgba.size();

        if(useFixedDownscale){
            adjustedSize = downscaleResolution;
        }else{
            adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
        }

        rgba.copyTo(workingMat);

        if(workingMat.empty()){
            return rgba;
        }
        Imgproc.resize(workingMat, workingMat,adjustedSize); // Downscale
        workingMat = MathFTC.crop(workingMat, cropTLCorner, cropBRCorner);

        Imgproc.resize(process(workingMat),workingMat,getInitSize()); // Process and scale back to original size for viewing

        return workingMat;
    }

    public Size getInitSize() {
        return initSize;
    }

    public Size getAdjustedSize() {
        return adjustedSize;
    }
}
