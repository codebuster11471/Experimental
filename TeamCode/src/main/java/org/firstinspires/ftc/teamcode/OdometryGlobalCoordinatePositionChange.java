package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.HardwareMecanum;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePositionChange implements Runnable{

    DcMotor motorFL, motorFR, motorRL, motorRR;
    //Thead run condition
    private boolean isRunning = true;


    //Position variables used for storage and calculations

    double encoderFL = 0, encoderFR = 0, encoderRL = 0, encoderRR = 0;  //Intialize encoder count
    double lastEncoderFL = 0, lastEncoderFR = 0, lastEncoderRL = 0, lastEncoderRR = 0;  //Intialize last encoder count
    double delta_y_r, delta_x_r, delta_theta_r, delta_y, delta_x, delta_theta, theta;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;


    //Algorithm constants
    double inchPerCount;
//    private double horizontalEncoderTickPerDegreeOffset;


    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
/**
    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
*/
    /**
     * Constructor for GlobalCoordinatePosition Thread
     * The stuff below is from the original build, kept because maybe useful? Probably not but wth.
     * //@param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * //@param verticalEncoderRight right odometry encoder, facing the vertical direction
     * //@param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePositionChange(DcMotor motorFL, DcMotor motorFR, DcMotor motorRL, DcMotor motorRR, double inchPerCount, int threadSleepDelay){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorRL = motorRL;
        this.motorRR = motorRR;
        this.inchPerCount = inchPerCount;
        sleepTime = threadSleepDelay;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalPositionUpdate(){
        // Get current encoder values
        encoderFL = motorFL.getCurrentPosition();
        encoderFR = motorFR.getCurrentPosition();
        encoderRL = motorRL.getCurrentPosition();
        encoderRR = motorRR.getCurrentPosition();

        //Calculate deltaPosition of all 4 wheels, convert to inches
        double deltaFL =(encoderFL-lastEncoderFL) * inchPerCount;
        double deltaFR =(encoderFR-lastEncoderFR) * inchPerCount;
        double deltaRL =(encoderRL-lastEncoderRL) * inchPerCount;
        double deltaRR =(encoderRR-lastEncoderRR) * inchPerCount;

        // Update lastEncoder values
        lastEncoderFL = encoderFL;
        lastEncoderFR = encoderFR;
        lastEncoderRL = encoderRL;
        lastEncoderRR = encoderRR;


        // Position update equations using 4-wheel mecanum inputs, see
        // https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Mobile_Robot_Kinematics_for_FTC.pdf
        delta_y_r = (deltaFrontRight+deltaFrontLeft+deltaBackLeft+deltaBackRight)/4.0;
        delta_x_r = (-deltaFrontRight+deltaFrontLeft-deltaBackLeft+deltaBackRight)/4.0;
        delta_theta_r = (deltaFrontRight-deltaFrontLeft-deltaBackLeft+deltaBackRight)/(4.0*LR); //lr is track width. Could probably use gyro

        delta_y = (delta_x_r*Math.sin(-theta)+delta_y_r*Math.cos(-theta));
        delta_x = (delta_x_r*Math.cos(-theta)-delta_y_r*Math.sin(-theta));
        delta_theta = delta_theta_r;
        robotGlobalXCoordinatePosition += delta_x;
        robotGlobalYCoordinatePosition += delta_y;
        robotOrientationRadians += delta_theta;
        if(theta>360){
            theta-=360;
        }
        if(theta<0){
            theta+=360;
        }
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){
        return robotGlobalXCoordinatePosition;
    }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){
        return robotGlobalYCoordinatePosition;
    }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){
        return Math.toDegrees(robotOrientationRadians) % 360;
    }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }
/**    public void reverseLeftEncoder(){
 if(verticalLeftEncoderPositionMultiplier == 1){
 verticalLeftEncoderPositionMultiplier = -1;
 }else{
 verticalLeftEncoderPositionMultiplier = 1;
 }
 }

 public void reverseRightEncoder(){
 if(verticalRightEncoderPositionMultiplier == 1){
 verticalRightEncoderPositionMultiplier = -1;
 }else{
 verticalRightEncoderPositionMultiplier = 1;
 }
 }

 public void reverseNormalEncoder(){
 if(normalEncoderPositionMultiplier == 1){
 normalEncoderPositionMultiplier = -1;
 }else{
 normalEncoderPositionMultiplier = 1;
 }
 }

**/
    /**
     * Runs the thread
     */
    @Override
    public void run() {

        while(isRunning) {
            globalPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
