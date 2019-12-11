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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="LeftFoundationDrag_ParkRight_Wall", group="Codebusters")
//@Disabled
public class LeftFoundationDrag_ParkRight_Wall extends LinearOpMode {

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
    boolean fangsClosed = false;  //Fang position tracker, FALSE = open, TRUE = closed

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

        //Initialize fangs
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        foundationFangs(0);

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
            pidDriveCommand(-32, 32, 0, 0.25, 3.5);
            pidDriveCommand(-36, 32, 0, 0.15, 1.5);
            foundationFangs(1);
            pidDriveCommand(0, 0, 0, 0, 1);
            pidDriveCommand(-4, 32, 0, 0.2, 8);
            foundationFangs(0);
            pidDriveCommand(0, 0, 0, 0, 1);
            pidDriveCommand(-6, 32, 0, 0.15, 2);
            pidDriveCommand(-6, -25, 0, 0.25, 6);
            pidDriveCommand(-6, -25, 180, 0.2, 5);
            break;
        }
        //Stop the odometry processing thread
//        odometryThread.interrupt();
    }

    public void foundationFangs(int fangOperationCmd) {
        if(fangOperationCmd == 1) {
            servoL.setPosition(0.33);
            servoR.setPosition(0.33);
        }
        else if(fangOperationCmd == 0) {
            servoL.setPosition(0);
            servoR.setPosition(0);
        }
    }


    public void pidDriveCommand(double xTarget, double yTarget, double thetaTarget, double maxPower, double timeout){
        //Start the odometry processing thread
        odometryPositionUpdate positionUpdate = new odometryPositionUpdate(motorFL, motorFR, motorRL, motorRR, inchPerCount, trackWidth, wheelBase, 75);
        Thread odometryThread = new Thread(positionUpdate);
        odometryThread.start();

        //PID controller declarations
        double Kp = 0.06;  //[--]
        double Ki = 0.00005;  //[--]
        double Kd = 0.008*0;  //[--]
        double strafeError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double driveError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double turnError = 1;  //[deg];  Initialize to 1 so it is larger than turnTol
        double strafeIntegral = 0;  //[in]
        double driveIntegral = 0;  //[in]
        double turnIntegral = 0;  //[deg]
        double strafeDerivative = 0;  //[in]
        double driveDerivative = 0;  //[in]
        double turnDerivative = 0;  //[deg]
        double prevStrafeError = 0;
        double prevDriveError = 0;
        double prevTurnError = 0;
        double strafeDriveTol = 0.1;  //[inch]; Allowable strafe/drive error before exiting PID loop
        double turnTol = 0.5;  //[deg]; Allowable turn error before exiting PID loop
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Output declarations
        double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
        double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00
        double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && moveComplete == false){
            //Get current positions
            absPosnX = startPosnX + positionUpdate.returnOdometryX();
            absPosnY = startPosnY + positionUpdate.returnOdometryY();
            absPosnTheta = startPosnTheta + positionUpdate.returnOdometryTheta();

            //Calculate error
            strafeError = yTarget - absPosnY;
            driveError = xTarget - absPosnX;
            turnError = thetaTarget - absPosnTheta;

            if (Math.abs(strafeError) < 1){
                strafeIntegral = strafeIntegral + strafeError*0.02;
            } else strafeIntegral = 0;

            if (Math.abs(driveError) < 1){
                driveIntegral = driveIntegral + driveError*0.02;
            } else driveIntegral = 0;

            if (Math.abs(turnError) < 1) {
                turnIntegral = turnIntegral + turnError*0.02;
            } else turnIntegral = 0;

            strafeDerivative = (strafeError - prevStrafeError)/0.02;
            driveDerivative = (driveError - prevDriveError)/0.02;
            turnDerivative = (turnError - prevTurnError)/0.02;

            prevStrafeError = strafeError;
            prevDriveError = driveError;
            prevTurnError = turnError;

            if(Math.abs(strafeError) < strafeDriveTol && Math.abs(driveError) < strafeDriveTol && Math.abs(turnError) < turnTol){
                moveComplete = true;
            }  //If robot is within specified drive/strafe/turn tolerances, exit the loop early, otherwise it will exit after a timeout

            //PID summation
            driveCmd = Kp*driveError + Ki*driveIntegral + Kd*driveDerivative;
            strafeCmd = Kp*strafeError + Ki*strafeIntegral + Kd*strafeDerivative;
            turnCmd = -Kp*turnError - Ki*turnIntegral - Kd*turnDerivative;

            //Clip values within maximum specified power range
            driveCmd = Range.clip(driveCmd, -maxPower, maxPower);
            strafeCmd = Range.clip(strafeCmd, -maxPower, maxPower);
            turnCmd = Range.clip(turnCmd, -maxPower, maxPower);

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
    }
}
