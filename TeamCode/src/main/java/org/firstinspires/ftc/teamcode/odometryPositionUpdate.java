package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Codebusters in 2019.
 */
public class odometryPositionUpdate implements Runnable {

    //Motors
    DcMotor motorFL, motorFR, motorRL, motorRR;  //[null]
    double trackWidth, wheelBase;  //[inch]

    //IMU declarations
    BNO055IMU imu;
    Orientation angles;

    //Position variables used for storage and calculations
    double encoderFL = 0, encoderFR = 0, encoderRL = 0, encoderRR = 0;  //[cnt];  Intialize encoder count = 0
    double lastEncoderFL = 0, lastEncoderFR = 0, lastEncoderRL = 0, lastEncoderRR = 0;  //[cnt];  Intialize last encoder count = 0
    double odometryX = 0, odometryY = 0, odometryTheta = 0;  //[inch];  Initialize at 0
    double dx =0, dxTemp = 0, dy =0, dyTemp = 0;  //[inch];  Storage variables

    //Algorithm constants
    double inchPerCount;  //[inch/cnt]

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;  //[msec]

    //Thead run condition
    private boolean isRunning = true;  //[bool]


    /**
     * Import values and assign locally.  Input units must be as specified
     */
    public odometryPositionUpdate(DcMotor motorFL, DcMotor motorFR, DcMotor motorRL, DcMotor motorRR, double inchPerCount, double trackWidth, double wheelBase, int threadSleepDelay){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorRL = motorRL;
        this.motorRR = motorRR;
        this.inchPerCount = inchPerCount;  //[inch/cnt]
        this.trackWidth = trackWidth;  //[inch]
        this.wheelBase = wheelBase;  //[inch]
        sleepTime = threadSleepDelay;  //[msec]
    }

    /**
     * Updates the delta (x, y, theta)  position of the robot using odometry input
     */
    private void odometryDeltaPositionUpdate(){
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

        //Update lastEncoder values
        lastEncoderFL = encoderFL;
        lastEncoderFR = encoderFR;
        lastEncoderRL = encoderRL;
        lastEncoderRR = encoderRR;

        //Position update equations using 4-wheel mecanum inputs, see
        //https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Mobile_Robot_Kinematics_for_FTC.pdf
        dx = (deltaFL + deltaFR + deltaRL + deltaRR)/4;  //[inch]
        dy = (-deltaFL + deltaFR + deltaRL - deltaRR)/4;  //[inch]

        //Angular correction, see Wikipedia topic "Rotation Matrix"
        dxTemp = dx*Math.cos(Math.toRadians(odometryTheta)) - dy*Math.sin(Math.toRadians(odometryTheta));
        dyTemp = dx*Math.sin(Math.toRadians(odometryTheta)) + dy*Math.cos(Math.toRadians(odometryTheta));

        //Final positions
        odometryX = odometryX + dxTemp;  //[inch]
        odometryY = odometryY + dyTemp;  //[inch]
        odometryTheta = odometryTheta + (-deltaFL + deltaFR - deltaRL + deltaRR)/4/(trackWidth/2+wheelBase/2);  //[inch]

        //Angular correction, see Wikipedia topic "Rotation Matrix"
//        odometryXtemp = odometryX*Math.cos(Math.toRadians(odometryTheta)) - odometryY*Math.sin(Math.toRadians(odometryTheta));
//        odometryYtemp = odometryX*Math.sin(Math.toRadians(odometryTheta)) + odometryY*Math.cos(Math.toRadians(odometryTheta));
//        odometryX = odometryXtemp;
//        odometryY = odometryYtemp;
    }

    /**
     * Returns the robot's delta x in inches
     */
    public double returnOdometryX(){
//        return -odometryY;  //[inch];  This is the absolute x position of the robot, but does not account for initial robot placement on the field.  Note!  Axis swap between X and Y
        return odometryX;  //[inch];  This is the absolute y position of the robot, but does not account for initial robot placement on the field.  Note!  Axis swap between X and Y

    }

    /**
     * Returns the robot's delta y in inches
     */
    public double returnOdometryY(){
//        return odometryX;  //[inch];  This is the absolute y position of the robot, but does not account for initial robot placement on the field.  Note!  Axis swap between X and Y
        return -odometryY;  //[inch];  This is the absolute x position of the robot, but does not account for initial robot placement on the field.  Note!  Axis swap between X and Y
    }

    /**
     * Returns the robot's delta orientation in degrees
     */
    public double returnOdometryTheta(){
        return Math.toDegrees(odometryTheta) % 360;  //[degrees];  This is the absolute heading of the robot, but does not account for initial robot placement on the field
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            odometryDeltaPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
