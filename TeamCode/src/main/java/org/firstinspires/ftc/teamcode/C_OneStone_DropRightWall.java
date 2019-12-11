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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="C_OneStone_DropRightWall", group="Codebusters")
//@Disabled
public class C_OneStone_DropRightWall extends LinearOpMode {

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

    //Fang Declarations
    Servo servoL;  //Left fang servo
    Servo   servoR;  //Right fang servo

    //Odometry declarations
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

    //IMU declarations
    BNO055IMU imu;
    Orientation angles;


    int skystoneLocation = -1;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        //Intialize the computer vision detector
        detector = new modifiedGoldDetector(); //Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.BACK); //Initialize it with the app context and camera
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

        //Initialize the intake/outtake
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(DcMotor.Direction.REVERSE);

        //Initialize fangs
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

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
            if(skystoneLocation == 1) {
                pidDriveCommand(26, -10, 0.25, 3);
                intakeOperation(1);
                pidDriveCommand(38, -10, 0.10, 3);
                pidDriveCommand(2, -10, 0.25, 4);
                pidDriveCommand(2, 58, 0.25, 6);
                intakeOperation(-1);
                pidDriveCommand(-1, -1, 0, 1);
                intakeOperation(0);
                pidDriveCommand(2, 36, 0.25, 3);
            }
            if(skystoneLocation == 2) {
                pidDriveCommand(26, 0,  0.25, 3);
                intakeOperation(1);
                pidDriveCommand(38, 0,  0.10, 3);
                pidDriveCommand(2, 0, 0.25, 4);
                pidDriveCommand(2, 58, 0.25, 6);
                intakeOperation(-1);
                pidDriveCommand(-1, -1,  0, 1);
                intakeOperation(0);
                pidDriveCommand(2, 36, 0.25, 3);
            }
            if(skystoneLocation == 3) {
                pidDriveCommand(26, 10, 0.25, 3);
                intakeOperation(1);
                pidDriveCommand(38, 10, 0.10, 3);
                pidDriveCommand(2, 10, 0.25, 4);
                pidDriveCommand(2, 58, 0.25, 6);
                intakeOperation(-1);
                pidDriveCommand(-1, -1,  0, 1);
                intakeOperation(0);
                pidDriveCommand(2, 36, 0.25, 3);
            }
            pidTurnCommand(0, 0.25, 2);
            break;
        }
        //Shutdown on STOP
        intakeOperation(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
        imu.close();
        if(detector != null) detector.disable();
    }


    public void intakeOperation(int intakeOperationCmd) {
        if(intakeOperationCmd == 1) {
            intakeL.setPower(1);
            intakeR.setPower(1);
        }
        else if(intakeOperationCmd == -1) {
            intakeL.setPower(-0.65);
            intakeR.setPower(-0.65);
        }
       else if(intakeOperationCmd == 0) {
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
    }


    public void pidDriveCommand(double xTarget, double yTarget, double maxPower, double timeout){
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
        double xIntegral = 0;  //[in]
        double yIntegral = 0;  //[in]
        double xDerivative = 0;  //[in]
        double yDerivative = 0;  //[in]
        double prevYError = 0;
        double prevXError = 0;
        double xyTol = 0.1;  //[inch]; Allowable strafe/drive error before exiting PID loop
        double driveCmdtemp = 0;  //Storage variable
        double strafeCmdtemp = 0;  //Storage variable
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Odometry declaration
        double odometryX = 0, odometryY = 0;  //[inch];  Initialize at 0

        //Output declarations
        double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
        double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && moveComplete == false) {
            //Get current positions
            odometryX = positionUpdate.returnOdometryX();
            odometryY = positionUpdate.returnOdometryY();
            absPosnX = odometryX;
            absPosnY = odometryY;

            xError = xTarget - absPosnX;
            yError = yTarget - absPosnY;

            if (Math.abs(xError) < 1) {  //Only enable integral when error is less than 1 inch
                xIntegral = xIntegral + xError*0.02;
            } else xIntegral = 0;
            if (Math.abs(yError) < 1) {  //Only enable integral when error is less than 1 inch
                yIntegral = yIntegral + yError*0.02;
            } else yIntegral = 0;

            xDerivative = (xError - prevXError)/0.02;
            yDerivative = (yError - prevYError)/0.02;

            prevXError = xError;
            prevYError = yError;

            if(Math.abs(yError) < xyTol && Math.abs(xError) < xyTol) {
                moveComplete = true;
            }  //If robot is within specified drive/strafe/turn tolerances, exit the loop early, otherwise it will exit after a timeout

            //PID summation
            driveCmd = Kp*xError + Ki*xIntegral + Kd*xDerivative;
            strafeCmd = Kp*yError + Ki*yIntegral + Kd*yDerivative;

            //Clip values within maximum specified power range
            driveCmd = Range.clip(driveCmd, -maxPower, maxPower);
            strafeCmd = Range.clip(strafeCmd, -maxPower, maxPower);

            //Angular correction, see Wikipedia topic "Rotation Matrix"
            driveCmdtemp = driveCmd*Math.cos(Math.toRadians(absPosnTheta)) - strafeCmd*Math.sin(Math.toRadians(absPosnTheta));
            strafeCmdtemp = driveCmd*Math.sin(Math.toRadians(absPosnTheta)) + strafeCmd*Math.cos(Math.toRadians(absPosnTheta));
            driveCmd = driveCmdtemp;
            strafeCmd = strafeCmdtemp;

            //Send calculated power to wheels using mecanum equations
            motorFL.setPower(driveCmd + strafeCmd);
            motorFR.setPower(driveCmd - strafeCmd);
            motorRL.setPower(driveCmd - strafeCmd);
            motorRR.setPower(driveCmd + strafeCmd);

            //Telemetry
            telemetry.addData("X Position [in]", absPosnX);
            telemetry.addData("Y Position [in]", absPosnY);
            telemetry.addData("Orientation [deg]", absPosnTheta);
            telemetry.addData("Drive", driveCmd);
            telemetry.addData("Strafe", strafeCmd);
            telemetry.update();
        }
        //Stop motors when complete
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
    }

    public void pidTurnCommand(double thetaTarget, double maxPower, double timeout){
        //PID controller declarations
        double Kp = 0.08;  //[--]
        double Ki = 0.000005;  //[--]
        double Kd = 0.0008;  //[--]
        double thetaError = 1;  //[deg];  Initialize to 1 so it is larger than turnTol
        double thetaIntegral = 0;  //[deg]
        double thetaDerivative = 0;  //[deg]
        double prevThetaError = 0;
        double thetaTol = 0.1;  //[deg]; Allowable turn error before exiting PID loop
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Output declarations
        double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && moveComplete == false) {
            //Get current positions
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            absPosnTheta = angles.firstAngle;
            if (thetaTarget >= 0) {  //This is needed since it will otherwise behave oddly near +/- 180deg
                if (absPosnTheta < 0) {
                    thetaError = -(thetaTarget - Math.abs(absPosnTheta));
                }
                else if (absPosnTheta >= 0) {
                    thetaError = thetaTarget - absPosnTheta;
                }
            }
            else if (thetaTarget < 0) {  //This is needed since it will otherwise behave oddly near +/- 180deg
                if (absPosnTheta <= 0) {
                    thetaError = thetaTarget - absPosnTheta;
                }
                else if (absPosnTheta > 0) {
                    thetaError = Math.abs(thetaTarget) - absPosnTheta;
                }
            }

            if (Math.abs(thetaError) < 5) {  //Only enable integral when error is less than 5 degrees
                thetaIntegral = thetaIntegral + thetaError*0.02;
            } else thetaIntegral = 0;

            thetaDerivative = (thetaError - prevThetaError)/0.02;

            prevThetaError = thetaError;

            if(Math.abs(thetaError) < thetaTol) {
                moveComplete = true;
            }  //If robot is within specified drive/strafe/turn tolerances, exit the loop early, otherwise it will exit after a timeout

            //PID summation
            turnCmd = -Kp*thetaError - Ki*thetaIntegral - Kd*thetaDerivative;

            //Clip values within maximum specified power range
            turnCmd = Range.clip(turnCmd, -maxPower, maxPower);

            //Send calculated power to wheels using mecanum equations
            motorFL.setPower(turnCmd);
            motorFR.setPower(-turnCmd);
            motorRL.setPower(turnCmd);
            motorRR.setPower(-turnCmd);

            //Telemetry
            telemetry.addData("Orientation [deg]", absPosnTheta);
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
