/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.skystone.modifiedGoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="A_ParkLeftWall", group="Codebusters")
//@Disabled
public class A_ParkLeftWall extends LinearOpMode {

    //Detector declaration
    private modifiedGoldDetector detector;

    //Motor declarations
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorRL = null;
    private DcMotor motorRR = null;

    //Intake/outtake system declarations
    private DcMotor intakeR = null;
    private DcMotor intakeL = null;

    //IMU declarations
    BNO055IMU imu;
    Orientation angles;

    //Odometry declarations
    double startPosnX = 0;  //Define robot position here
    double startPosnY = 0;  //Define robot position here
    double startPosnTheta = 0;  //Define robot position here
    double absPosnX = 0;  //Absolute x position storage variable
    double absPosnY = 0;  //Absolute y position storage variable
    double absPosnTheta = 0;  //Absolute theta position storage variable

    static final double trackWidth = 14.75;  //Left-right distance between wheels (in inches)
    static final double wheelBase = 11.75;  //Front-back distance between wheels (in inches)
    static final double countsPerMotorRev = 537.6;  //Andymark Orbital 20
    static final double driveGearReduction = 1.0;  //This is < 1.0 if geared UP
    static final double slipFactor = 1.0;  //TODO
    static final double wheelDiameter = 3.0;  //Wheel diameter (in inches)
    static final double countsPerInch = (countsPerMotorRev * driveGearReduction) / (wheelDiameter * 3.1415);  //Encoder counts per inch of travel
    static final double inchPerCount = (wheelDiameter * 3.1415) / (countsPerMotorRev * driveGearReduction);  //Inches of travel per encoder count

    int skystoneLocation = -1;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        //Intialize the computer vision detector
        detector = new modifiedGoldDetector(); //Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.FRONT); //Initialize it with the app context and camera
        detector.enable(); //Start the detector

        //Initialize the drivetrain
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorRL = hardwareMap.get(DcMotor.class, "motorRL");
        motorRR = hardwareMap.get(DcMotor.class, "motorRR");
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorRL.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setDirection(DcMotor.Direction.REVERSE);

        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(DcMotor.Direction.REVERSE);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Reset encoders
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("IsFound: ", detector.isFound());
        telemetry.addData(">", "Waiting for start");
        telemetry.update();

        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        skystoneLocation = detector.isFound();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Disable the detector
        if(detector != null) detector.disable();


        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pidDriveCommand(3, 0, 0, 0.25, 5);
            pidDriveCommand(3, -24, 0, 0.25, 5);
            pidDriveCommand(0, -24, 0, 0.25, 5);
            break;
        }
        //Stop the odometry processing thread
//        odometryThread.interrupt();
    }


    public void pidDriveCommand(double xTarget, double yTarget, double thetaTarget, double maxPower, double timeout){  //Function for drive/strafe, and to maintain heading
        //Start the odometry processing thread
        odometryPositionUpdate positionUpdate = new odometryPositionUpdate(motorFL, motorFR, motorRL, motorRR, inchPerCount, trackWidth, wheelBase, 75);
        Thread odometryThread = new Thread(positionUpdate);
        odometryThread.start();

        //PID controller declarations
        double Kp = 0.08;  //[--]
        double Ki = 0.000005;  //[--]
        double Kd = 0.0008;  //[--]
        double xError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double yError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double thetaError = 1;  //[deg];  Initialize to 1 so it is larger than turnTol
        double xIntegral = 0;  //[in]
        double yIntegral = 0;  //[in]
        double thetaIntegral = 0;  //[deg]
        double xDerivative = 0;  //[in]
        double yDerivative = 0;  //[in]
        double thetaDerivative = 0;  //[deg]
        double prevYError = 0;
        double prevXError = 0;
        double prevThetaError = 0;
        double xyTol = 0.1;  //[inch]; Allowable strafe/drive error before exiting PID loop
        double thetaTol = 0.5;  //[deg]; Allowable turn error before exiting PID loop
        double driveCmdtemp = 0;  //Storage variable
        double strafeCmdtemp = 0;  //Storage variable
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Odometry declaration
        double odometryX = 0, odometryY = 0, odometryTheta = 0;  //[inch];  Initialize at 0
        double odometryXtemp =0, odometryYtemp = 0;  //[inch];  Storage variables

        //Output declarations
        double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
        double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00
        double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && moveComplete == false) {
        //Get current positions, apply rotation matrix correction
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);  //Use IMU for angular heading
        absPosnX = positionUpdate.returnOdometryX();
        absPosnY = positionUpdate.returnOdometryY();
        absPosnTheta = angles.firstAngle;

        xError = xTarget - absPosnX;
        yError = yTarget - absPosnY;
        thetaError = thetaTarget - absPosnTheta;  //thetaError only used to maintain heading, not for making actual turns

        if (Math.abs(xError) < 1) {  //Only enable integral when error is less than 1 inch
            xIntegral = xIntegral + xError*0.02;
        } else xIntegral = 0;
        if (Math.abs(yError) < 1) {  //Only enable integral when error is less than 1 inch
            yIntegral = yIntegral + yError*0.02;
        } else yIntegral = 0;
        if (Math.abs(thetaError) < 1) {  //Only enable integral when error is less than 1 inch
            thetaIntegral = thetaIntegral + thetaError*0.02;
        } else thetaIntegral = 0;

        xDerivative = (xError - prevXError)/0.02;
        yDerivative = (yError - prevYError)/0.02;
        thetaDerivative = (thetaError - prevThetaError)/0.02;

        prevXError = xError;
        prevYError = yError;
        prevThetaError = thetaError;

        if(Math.abs(yError) < xyTol && Math.abs(xError) < xyTol && Math.abs(thetaError) < thetaTol) {
            moveComplete = true;
        }  //If robot is within specified drive/strafe/turn tolerances, exit the loop early, otherwise it will exit after a timeout

        //PID summation
        driveCmd = Kp*xError + Ki*xIntegral + Kd*xDerivative;
        strafeCmd = Kp*yError + Ki*yIntegral + Kd*yDerivative;
        turnCmd = 0.5*(-Kp*thetaError - Ki*thetaIntegral - Kd*thetaDerivative);

        //Clip values within maximum specified power range
        driveCmd = Range.clip(driveCmd, -maxPower, maxPower);
        strafeCmd = Range.clip(strafeCmd, -maxPower, maxPower);
        turnCmd = Range.clip(turnCmd, -maxPower, maxPower);

        //Angular correction, see Wikipedia topic "Rotation Matrix"
        driveCmdtemp = driveCmd*Math.cos(Math.toRadians(absPosnTheta)) - strafeCmd*Math.sin(Math.toRadians(absPosnTheta));
        strafeCmdtemp = driveCmd*Math.sin(Math.toRadians(absPosnTheta)) + strafeCmd*Math.cos(Math.toRadians(absPosnTheta));
        driveCmd = driveCmdtemp;
        strafeCmd = strafeCmdtemp;

        //Send calculated power to wheels using mecanum equations
        motorFL.setPower(driveCmd + strafeCmd + turnCmd);
        motorFR.setPower(driveCmd - strafeCmd - turnCmd);
        motorRL.setPower(driveCmd - strafeCmd + turnCmd);
        motorRR.setPower(driveCmd + strafeCmd - turnCmd);

        //Telemetry
        telemetry.addData("X Position [in]", absPosnX);
        telemetry.addData("Y Position [in]", absPosnY);
        telemetry.addData("Orientation [deg]", absPosnTheta);
        telemetry.addData("Drive", driveCmd);
        telemetry.addData("Strafe", strafeCmd);
        telemetry.addData("Turn", turnCmd);
        telemetry.update();
    }
    //Stop motors when complete
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
}
}
