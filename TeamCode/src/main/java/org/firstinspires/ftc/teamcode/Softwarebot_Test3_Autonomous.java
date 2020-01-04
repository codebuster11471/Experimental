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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous Test3", group="Codebusters")
@Disabled
public class Softwarebot_Test3_Autonomous extends LinearOpMode {

    //Detector declaration
    private modifiedGoldDetector detector;

    //Motor declarations
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorRL = null;
    private DcMotor motorRR = null;

    //Odometry declarations
    double startPosnX = 0;  //Define robot position here
    double startPosnY = 0;  //Define robot position here
    double startPosnTheta = 0;  //Define robot position here
    double absPosnX = 0;  //Absolute x position storage variable
    double absPosnY = 0;  //Absolute y position storage variable
    double absPosnTheta = 0;  //Absolute theta position storage variable

    static final double trackWidth = 16;  //TODO Left-right distance between wheels (in inches)
    static final double wheelBase = 16;  //TODO Front-back distance between wheels (in inches)
    static final double countsPerMotorRev = 1120;  //TODO Pull from motor specifications
    static final double driveGearReduction = 1.0;  //This is < 1.0 if geared UP
    static final double slipFactor = 1.0;  //TODO
    static final double wheelDiameter = 4.0;  //Wheel diameter (in inches)
    static final double countsPerInch = (countsPerMotorRev * driveGearReduction) / (wheelDiameter * 3.1415);  //Encoder counts per inch of travel
    static final double inchPerCount = (wheelDiameter * 3.1415) / (countsPerMotorRev * driveGearReduction);  //Inches of travel per encoder count

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


        //TODO Pass skystone location to storage

        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Disable the detector
        if(detector != null) detector.disable();

        //Start the odometry processing thread
        odometryPositionUpdate positionUpdate = new odometryPositionUpdate(motorFL, motorFR, motorRL, motorRR, inchPerCount, trackWidth, wheelBase, 75);
        Thread odometryThread = new Thread(positionUpdate);
        odometryThread.start();
        runtime.reset();

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            absPosnX = startPosnX + positionUpdate.returnOdometryX();
            absPosnY = startPosnY + positionUpdate.returnOdometryY();
            absPosnTheta = startPosnTheta + positionUpdate.returnOdometryTheta();

            telemetry.addData("X Position [in]", absPosnX);
            telemetry.addData("Y Position [in]", absPosnY);
            telemetry.addData("Orientation [deg]", absPosnTheta);
            telemetry.addData("Thread Active", odometryThread.isAlive());
            telemetry.update();

//Debug:  Drive forward for 3.0 seconds and make sure telemetry is correct
            if (runtime.seconds() < 3.0) {
                motorFL.setPower(0.25);
                motorFR.setPower(0.25);
                motorRL.setPower(0.25);
                motorRR.setPower(0.25);
            }else {
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorRL.setPower(0);
                motorRR.setPower(0);
            }


        }
        //Stop the odometry processing thread
        odometryThread.interrupt();
    }
}
