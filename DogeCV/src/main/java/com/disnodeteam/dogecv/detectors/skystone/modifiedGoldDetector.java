package com.disnodeteam.dogecv.detectors.skystone;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.DogeCVDetector_Modified;
import com.disnodeteam.dogecv.math.MathFTC;

import org.opencv.core.Core;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;



/**
 * Created by Victo on 9/10/2018.
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
    private boolean found    = false; // Is the gold mineral found
    private Point   screenPosition = new Point(); // Screen position of the mineral
    private Rect    foundRect = new Rect(); // Found rect
    private int skylocation = 4;

//define analysis zones
    public Point sector1TLcorner = new Point(0, 160); //Sets the top left corner of first sector in pixel (x,y) coordinates
    public Point sector1BRcorner = new Point(213, 320); //Sets the bottom right corner of first sector in pixel (x,y) coordinates
    public Point sector2TLcorner = new Point(213, 160); //Sets the top left corner of second sector in pixel (x,y) coordinates
    public Point sector2BRcorner = new Point(426, 320); //Sets the bottom right corner of second sector in pixel (x,y) coordinates
    public Point sector3TLcorner = new Point(426, 160); //Sets the top left corner of third sector in pixel (x,y) coordinates
    public Point sector3BRcorner = new Point(640, 320); //Sets the bottom right corner of third sector in pixel (x,y) coordinates

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


    @Override
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
        double sector2RMeanSrc = sector2Mean.get(0, 0)[0] * 2;
        double sector2GMeanSrc = sector2Mean.get(1, 0)[0] * 2;
        double sector2BMeanSrc = sector2Mean.get(2, 0)[0] * 2;
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
            skylocation = 1;
        } else if (sector3RMeanSrc < sector1RMeanSrc && sector3RMeanSrc < sector2RMeanSrc) {
            skylocation = 3;
        } else
            skylocation = 2;

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

        Imgproc.putText(displayMat,"skystone_at: "+ skylocation, new Point(0,getAdjustedSize().height - 420),0,1, new Scalar(255,0,0),2);


        // Current result
        Rect bestRect = null;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

//        // Loop through the contours and score them, searching for the best result
//        for(MatOfPoint cont : contoursYellow){
//            double score = calculateScore(cont); // Get the difference score using the scoring API
//
//            // Get bounding rect of contour
//            Rect rect = Imgproc.boundingRect(cont);
//            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect
//
//            // If the result is better then the previously tracked one, set this rect as the new best
//            if(score < bestDifference){
//                bestDifference = score;
//                bestRect = rect;
//            }
//        }

        if(bestRect != null){
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        }else{
            found = false;
        }


        //Print result
//        Imgproc.putText(displayMat,"R: " + sector1RMeanSrc,new Point(0,getAdjustedSize().height - 30),0,1, new Scalar(255,0,0),2);
//        Imgproc.putText(displayMat,"G: " + sector1GMeanSrc,new Point(0,getAdjustedSize().height - 130),0,1, new Scalar(255,0,0),2);
//        Imgproc.putText(displayMat,"B: " + sector1BMeanSrc,new Point(0,getAdjustedSize().height - 230),0,1, new Scalar(255,0,0),2);


        return displayMat;
    }

    @Override
    public void useDefaults() {
//        addScorer(ratioScorer);
//
//        // Add diffrent scoreres depending on the selected mode
//        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
//            addScorer(maxAreaScorer);
//        }
//
//        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
//            addScorer(perfectAreaScorer);
//        }
//
    }

    /**
     * Returns the gold element's last position in screen pixels
     * @return position in screen pixels
     */
    public Point getScreenPosition(){
        return screenPosition;
    }

    /**
     * Returns the gold element's found rectangle
     * @return gold element rect
     */
    public Rect getFoundRect() {
        return foundRect;
    }

    /**
     * Returns if a gold mineral is being tracked/detected
     * @return if a gold mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
    }
}
