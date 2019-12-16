package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Codebusters in 2019.
 */
public class deadwheelPositionUpdate implements Runnable {

    //Motors
    DcMotor deadwheelX, deadwheelY;  //[null]

    //IMU declarations
    Orientation angles;

    //Position variables used for storage and calculations
    double encoderX = 0, encoderY;  //[cnt];  Intialize encoder count = 0
    double positionX = 0, positionY, theta = 0;
//    double dx =0, dxTemp = 0, dy =0, dyTemp = 0;  //[inch];  Storage variables

    //Algorithm constants
    double inchPerCount;  //[inch/cnt]

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;  //[msec]

    //Thead run condition
    private boolean isRunning = true;  //[bool]


    /**
     * Import values and assign locally.  Input units must be as specified
     */
    public deadwheelPositionUpdate(DcMotor deadwheelX, DcMotor deadwheelY, BNO055IMU imu, double inchPerCount,int threadSleepDelay){
        this.deadwheelX = deadwheelX;
        this.deadwheelY = deadwheelY;
        this.angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.inchPerCount = inchPerCount;  //[inch/cnt]
        sleepTime = threadSleepDelay;  //[msec]
    }

    /**
     * Updates the delta (x, y, theta)  position of the robot using odometry input
     */
    private void deadwheelDeltaPositionUpdate(){

        theta = angles.firstAngle;

        // Get current encoder values
        encoderX = deadwheelX.getCurrentPosition();
        encoderY = deadwheelY.getCurrentPosition();

//        //Update lastEncoder values
//        lastEncoderX = encoderX;
//        lastEncoderY = encoderY;

//        //Angular correction, see Wikipedia topic "Rotation Matrix"
//        dxTemp = dx*Math.cos(Math.toRadians(odometryTheta)) - dy*Math.sin(Math.toRadians(odometryTheta));
//        dyTemp = dx*Math.sin(Math.toRadians(odometryTheta)) + dy*Math.cos(Math.toRadians(odometryTheta));

        //Final positions
        positionX = encoderX * inchPerCount;  //[inch]
        positionY = encoderY * inchPerCount;  //[inch]
    }

    /**
     * Returns the robot's delta x (front-back) in inches
     */
    public double returnPositionX(){
       return positionX;  //[inch];  This is the absolute x (front-back) position of the robot, but does not account for initial robot placement on the field.

    }

    /**
     * Returns the robot's y-position (left-right) in inches
     */
    public double returnPositionY(){
       return positionY;  //[inch];  This is the absolute y (left-right) position of the robot, but does not account for initial robot placement on the field.
    }

    /**
     * Returns the robot's heading in degrees
     */
    public double returnTheta(){
        return theta;  //[inch];  This is the absolute heading angle of the robot, but does not account for initial robot placement on the field.
    }


    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            deadwheelDeltaPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
