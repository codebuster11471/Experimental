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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Autonomous Test4", group="Codebusters")
@Disabled
public class Softwarebot_Test4_Autonomous extends LinearOpMode {

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

    //Odometry declarations
    double startPosnX = 0;  //Define robot position here
    double startPosnY = 0;  //Define robot position here
    double startPosnTheta = 0;  //Define robot position here
    double absPosnX = 0;  //Absolute x position storage variable
    double absPosnY = 0;  //Absolute y position storage variable
    double absPosnTheta = 0;  //Absolute theta position storage variable

    static final double trackWidth = 4;  //TODO Left-right distance between wheels (in inches)
    static final double wheelBase = 4;  //TODO Front-back distance between wheels (in inches)
    static final double countsPerMotorRev = 1120;  //TODO Pull from motor specifications
    static final double driveGearReduction = 1.0;  //This is < 1.0 if geared UP
    static final double slipFactor = 1.0;  //TODO
    static final double wheelDiameter = 4.0;  //Wheel diameter (in inches)
    static final double countsPerInch = (countsPerMotorRev * driveGearReduction) / (wheelDiameter * 3.1415);  //Encoder counts per inch of travel
    static final double inchPerCount = (wheelDiameter * 3.1415) / (countsPerMotorRev * driveGearReduction);  //Inches of travel per encoder count

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
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);

        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.FORWARD);


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
            intakeL.setPower(1);
            intakeR.setPower(-1);
            pidDriveCommand(-1, -1, 0, 0.00, 0.5);
            intakeL.setPower(0);
            intakeR.setPower(0);

            if(skystoneLocation == 1) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
                pidDriveCommand(-16, 36, 0, 0.55, 4);
                intakeL.setPower(0);
                intakeR.setPower(0);
                pidDriveCommand(-16, 24, 0, 0.55, 2);
                pidDriveCommand(-48, 24, 0, 0.55, 4);
                intakeL.setPower(1);
                intakeR.setPower(1);
                pidDriveCommand(-1, -1, 0, 0.00, 2);
            }
            if(skystoneLocation == 2) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
                pidDriveCommand(0, 36, 0, 0.55, 4);
                intakeL.setPower(0);
                intakeR.setPower(0);
                pidDriveCommand(0, 24, 0, 0.55, 2);
                pidDriveCommand(-48, 24, 0, 0.55, 4);
                intakeL.setPower(1);
                intakeR.setPower(1);
                pidDriveCommand(-1, -1, 0, 0.00, 2);
            }
            if(skystoneLocation == 3) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
                pidDriveCommand(16, 36, 0, 0.55, 4);
                intakeL.setPower(0);
                intakeR.setPower(0);
                pidDriveCommand(16, 24, 0, 0.55, 2);
                pidDriveCommand(-48, 24, 0, 0.55, 4);
                intakeL.setPower(1);
                intakeR.setPower(1);
                pidDriveCommand(-1, -1, 0, 0.00, 2);
            }
            break;
        }
        intakeL.setPower(0);
        intakeR.setPower(0);
        //Stop the odometry processing thread
//        odometryThread.interrupt();
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
        double turnTol = 0.1;  //[deg]; Allowable turn error before exiting PID loo
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Output declarations
        double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
        double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00
        double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout){
            //Get current positions
            absPosnX = startPosnX + positionUpdate.returnOdometryX();
            absPosnY = startPosnY + positionUpdate.returnOdometryY();
            absPosnTheta = startPosnTheta + positionUpdate.returnOdometryTheta();

            //Calculate error
            strafeError = xTarget - absPosnX;
            driveError = yTarget - absPosnY;
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
