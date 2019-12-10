package com.disnodeteam.dogecv.detectors.skystone;

import com.disnodeteam.dogecv.detectors.DogeCVDetector_Modified;
import com.disnodeteam.dogecv.math.MathFTC;

import org.opencv.core.Core;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;



/**
 * Created by Victo on 9/10/2018.
 * Updated October 2019 Codebusters 11471
 */

public class modifiedGoldDetector extends DogeCVDetector_Modified {

    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat sector1crop = new Mat();
    private Mat sector2crop = new Mat();
    private Mat sector3crop = new Mat();
    private Mat sector1Mat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat sector2Mat = new Mat(); // Used for pre-processing and working with (blurring as an example)
    private Mat sector3Mat = new Mat(); // Used for pre-processing and working with (blurring as an example)


    // Results of the detector
    public int skystoneLocation = -1;

//define analysis zones
    public Point sector1TLcorner = new Point(159, 213); //Sets the top left corner of first sector in pixel (x,y) coordinates
    public Point sector1BRcorner = new Point(266, 300); //Sets the bottom right corner of first sector in pixel (x,y) coordinates
    public Point sector2TLcorner = new Point(266, 213); //Sets the top left corner of second sector in pixel (x,y) coordinates
    public Point sector2BRcorner = new Point(373, 300); //Sets the bottom right corner of second sector in pixel (x,y) coordinates
    public Point sector3TLcorner = new Point(373, 213); //Sets the top left corner of third sector in pixel (x,y) coordinates
    public Point sector3BRcorner = new Point(480, 300); //Sets the bottom right corner of third sector in pixel (x,y) coordinates

//define storage variables
    MatOfDouble sector1Mean = new MatOfDouble();
    MatOfDouble sector1stdev = new MatOfDouble();
    MatOfDouble sector2Mean = new MatOfDouble();
    MatOfDouble sector2stdev = new MatOfDouble();
    MatOfDouble sector3Mean = new MatOfDouble();
    MatOfDouble sector3stdev = new MatOfDouble();

    /**
     * Simple constructor
     */
    public modifiedGoldDetector() {
        super();
        detectorName = "Gold Detector"; // Set the detector name
    }


    public Mat process(Mat input) {
        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(sector1Mat);
        input.copyTo(sector2Mat);
        input.copyTo(sector3Mat);
        input.release();

        //Break image into 3 sectors, as defined by top-left and bottom-right corners
        sector1crop = MathFTC.crop(sector1Mat, sector1TLcorner, sector1BRcorner);
        sector2crop = MathFTC.crop(sector2Mat, sector2TLcorner, sector2BRcorner);
        sector3crop = MathFTC.crop(sector3Mat, sector3TLcorner, sector3BRcorner);
        //Determine mean color of sector 1 (RGB) and break into storage variables
        Core.meanStdDev(sector1crop, sector1Mean, sector1stdev);
        double sector1RMeanSrc = sector1Mean.get(0, 0)[0];
        double sector1GMeanSrc = sector1Mean.get(1, 0)[0];
        double sector1BMeanSrc = sector1Mean.get(2, 0)[0];
        //Determine mean color of sector 2 (RGB) and break into storage variables
        Core.meanStdDev(sector2crop, sector2Mean, sector2stdev);
        double sector2RMeanSrc = sector2Mean.get(0, 0)[0];
        double sector2GMeanSrc = sector2Mean.get(1, 0)[0];
        double sector2BMeanSrc = sector2Mean.get(2, 0)[0];
        //Determine mean color of sector 3 (RGB) and break into storage variables
        Core.meanStdDev(sector3crop, sector3Mean, sector3stdev);
        double sector3RMeanSrc = sector3Mean.get(0, 0)[0];
        double sector3GMeanSrc = sector3Mean.get(1, 0)[0];
        double sector3BMeanSrc = sector3Mean.get(2, 0)[0];
        //Release sector crops for memory
        sector1crop.release();
        sector2crop.release();
        sector3crop.release();

        //define skystone as the lowest value of red
        if (sector1RMeanSrc < sector3RMeanSrc && sector1RMeanSrc < sector2RMeanSrc) {
            skystoneLocation = 1;
        } else if (sector3RMeanSrc < sector1RMeanSrc && sector3RMeanSrc < sector2RMeanSrc) {
            skystoneLocation = 3;
        } else
            skystoneLocation = 2;

        //Draw rectangles around the three sectors
        Imgproc.rectangle(displayMat, sector1TLcorner, sector1BRcorner, new Scalar(200,0,255),4); // Draw rect for sector1
        Imgproc.rectangle(displayMat, sector2TLcorner, sector2BRcorner, new Scalar(200,0,255),4); // Draw rect for sector2
        Imgproc.rectangle(displayMat, sector3TLcorner, sector3BRcorner, new Scalar(200,0,255),4); // Draw rect for sector3

        //Label the RGB values of each sector
        Imgproc.putText(displayMat, String.format("R:  %3.1f", sector1RMeanSrc), new Point(0,getAdjustedSize().height - 330),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("G:  %3.1f", sector1GMeanSrc), new Point(0,getAdjustedSize().height - 360),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("B:  %3.1f", sector1BMeanSrc), new Point(0,getAdjustedSize().height - 390),0,1, new Scalar(255,0,0),2);

        Imgproc.putText(displayMat, String.format("R:  %3.1f", sector2RMeanSrc), new Point(213,getAdjustedSize().height - 330),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("G:  %3.1f", sector2GMeanSrc), new Point(213,getAdjustedSize().height - 360),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("B:  %3.1f", sector2BMeanSrc), new Point(213,getAdjustedSize().height - 390),0,1, new Scalar(255,0,0),2);

        Imgproc.putText(displayMat, String.format("R:  %3.1f", sector3RMeanSrc), new Point(426,getAdjustedSize().height - 330),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("G:  %3.1f", sector3GMeanSrc), new Point(426,getAdjustedSize().height - 360),0,1, new Scalar(255,0,0),2);
        Imgproc.putText(displayMat, String.format("B:  %3.1f", sector3BMeanSrc), new Point(426,getAdjustedSize().height - 390),0,1, new Scalar(255,0,0),2);


        if(skystoneLocation == 1){
            Imgproc.putText(displayMat,"X", new Point(173,getAdjustedSize().height - 185),0,5, new Scalar(200,0,255),10);
        }else if (skystoneLocation == 2){
            Imgproc.putText(displayMat,"X", new Point(270,getAdjustedSize().height - 185),0,5, new Scalar(200,0,255),10);
        }else if (skystoneLocation == 3){
            Imgproc.putText(displayMat,"X", new Point(367,getAdjustedSize().height - 185),0,5, new Scalar(200,0,255),10);
        }

        return displayMat;
    }

    public int isFound() {
        return skystoneLocation;
    }
}
