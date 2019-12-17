package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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


@Autonomous(name="D_Two_Stone_DropLeftBridge", group="Codebusters")
//@Disabled
public class D_Two_Stone_DropLeftBridge extends LinearOpMode {
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
    //Deadwheel declarations
    private DcMotor deadwheelX = null;
    private DcMotor deadwheelY = null;
    //Odometry declarations
    double positionX = 0, positionY, theta = 0;  //Storage variables, the current value
    double lastEncoderX = 0, lastEncoderY = 0, lastTheta = 0;  //Storage variables, the previous value
    static final double countsPerMotorRev = 2400;  //Signswise encoder in quadrature mode; 600ppr*4 = 2400cpr
    static final double driveGearReduction = 1.0;  //This is < 1.0 if geared UP
    static final double wheelDiameter = 2.0;  //Wheel diameter (in inches)
    static final double countsPerInch = (countsPerMotorRev * driveGearReduction) / (wheelDiameter * 3.1415);  //Encoder counts per inch of travel
    static final double inchPerCount = (wheelDiameter * 3.1415) / (countsPerMotorRev * driveGearReduction);  //Inches of travel per encoder count
    double x1 = -1.25, y1 = -3.25;  //[in];  x-deadwheel position relative to robot center
    double x2 = 0.125, y2 = 4.125;  //[in];  y-deadwheel position relative to robot center
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
        //Initialize deadwheels
        deadwheelX = hardwareMap.get(DcMotor.class, "deadwheelX");
        deadwheelY = hardwareMap.get(DcMotor.class, "deadwheelY");
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
        deadwheelX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadwheelY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Waiting for start");
        telemetry.update();
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
                pidDriveCommand(30, -20, -30, 0.6, 5); //1st position
                intakeOperation(1); //Intake on
                pidDriveCommand(40, -15,  -30, 0.3, 5);  //1st stone grab
                pidDriveCommand(26, -15, 0, 0.6, 5);  //1st backup
                pidDriveCommand(25, -50, 0, 0.75, 5);  //1st bridge
                intakeOperation(-1); //Intake out
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //1st drop off
                pidDriveCommand(25, 10, 0, 0.75, 5);  //2nd position
                intakeOperation(1); //Intake on
                pidDriveCommand(35, 10,  -30, 0.3, 5);  //2nd stone grab
                pidDriveCommand(23, 5, 0, 0.6, 5);  //2nd backup
                pidDriveCommand(25, -54, -10, 0.75, 5); //2nd bridge
                intakeOperation(-1); //Intake out
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //2nd drop off
                pidDriveCommand(24, -35, 0, 0.4, 5);  //Alliance park
                intakeOperation(0); //Intake off
            }
            if(skystoneLocation == 2) {
                pidDriveCommand(24, -8, -45, 0.35, 5); //Drive forward (center skystone)
                intakeOperation(1); //Intake on
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //Wait a little bit
                pidDriveCommand(40, -8, -45, 0.35, 5);  //Approach skystone
                pidDriveCommand(18, -8, 0, 0.35, 5);  //Backup
                pidDriveCommand(18, -52, 0, 0.35, 5);  //Drive under bridge
                intakeOperation(-1);  //Outtake on
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //Wait a little bit
                intakeOperation(0);  //Turn off intake/outtake
                pidDriveCommand(18, -30, 0, 0.35, 5);  //Park under bridge
            }
            if(skystoneLocation == 3) {
                pidDriveCommand(24, 0, -45, 0.35, 5); //Drive forward (center skystone)
                intakeOperation(1); //Intake on
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //Wait a little bit
                pidDriveCommand(40, 0, -45, 0.35, 5);  //Approach skystone
                pidDriveCommand(18, 0, 0, 0.35, 5);  //Backup
                pidDriveCommand(18, -52, 0, 0.35, 5);  //Drive under bridge
                intakeOperation(-1);  //Outtake on
                pidDriveCommand(-1, -1,  -1, 0, 0.5);  //Wait a little bit
                intakeOperation(0);  //Turn off intake/outtake
                pidDriveCommand(18, -30, 0, 0.35, 5);  //Park under bridge
            }
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


    public void foundationFangs(int fangOperationCmd) {  //Function to open/close fangs
        if(fangOperationCmd == 1) {
            servoL.setPosition(0.33);
            servoR.setPosition(0.33);
        }
        else if(fangOperationCmd == 0) {
            servoL.setPosition(0);
            servoR.setPosition(0);
        }
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


    public void deadwheelPositionUpdate() {
        double encoderX = 0, encoderY = 0;
        double dx = 0, dy = 0, dTheta;
        double xTemp = 0, yTemp = 0;

        //Read/poll current data positions
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        theta = angles.firstAngle;
        encoderX = deadwheelX.getCurrentPosition();
        encoderY = deadwheelY.getCurrentPosition();

        //x/y position update equations using 2-wheel + IMU.  See
        //https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Mobile_Robot_Kinematics_for_FTC.pdf
        dTheta = theta - lastTheta;
        dx =(encoderX-lastEncoderX) * inchPerCount - y1*(Math.toRadians(dTheta));
        dy =(encoderY-lastEncoderY) * inchPerCount - x2*(Math.toRadians(dTheta));

        //Update lastEncoder values
        lastEncoderX = encoderX;
        lastEncoderY = encoderY;
        lastTheta = theta;

        //Angular correction, see Wikipedia topic "Rotation Matrix"
        xTemp = dx*Math.cos(Math.toRadians(theta)) -   dy*Math.sin(Math.toRadians(theta));
        yTemp = dx*Math.sin(Math.toRadians(theta)) + dy*Math.cos(Math.toRadians(theta));  //Note sign change on cos term
        dx = xTemp;
        dy = -yTemp;  //Note sign change

        positionX = positionX + dx;  //[inch]
        positionY = positionY + dy;  //[inch]

        //Telemetry
        telemetry.addData("x Position =", positionX);
        telemetry.addData("y Position =", positionY);
        telemetry.addData("Heading =", theta);
        telemetry.update();
    }


    public void pidDriveCommand(double xTarget, double yTarget, double thetaTarget, double maxPower, double timeout){
        //PID controller declarations
        double Kp = (1-maxPower)/6;  //[--];  Kp is dependent on the maxPower input
        double Ki = 0.000005;  //[--]
        double Kd = 0.0008;  //[--]
        double xError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double yError = 1;  //[in];  Initialize to 1 so it is larger than strafeDriveTol
        double thetaError = 1;
        double xIntegral = 0;  //[in]
        double yIntegral = 0;  //[in]
        double thetaIntegral = 0;  //[deg]
        double xDerivative = 0;  //[in]
        double yDerivative = 0;  //[in]
        double thetaDerivative = 0;  //[deg]
        double prevYError = 0;
        double prevXError = 0;
        double prevThetaError = 0;
        double xyTol = 0.25;  //[inch]; Allowable strafe/drive error before exiting PID loop
        double thetaTol = 0.1;  //[deg]; Allowable turn error before exiting PID loop
        double driveCmdtemp = 0;  //Storage variable
        double strafeCmdtemp = 0;  //Storage variable
        boolean moveComplete = false;  //[bool];  Tracker to determine when movement is complete or not

        //Output declarations
        double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
        double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00
        double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout && moveComplete == false) {
            deadwheelPositionUpdate();

            xError = xTarget - positionX;
            yError = yTarget - positionY;
            thetaError = thetaTarget - theta;
//            if (thetaTarget >= 0) {  //This is needed since it will otherwise behave oddly near +/- 180deg
//                if (theta < 0) {
//                    thetaError = -(thetaTarget - Math.abs(theta));
//                }
//                else if (theta >= 0) {
//                    thetaError = thetaTarget - theta;
//                }
//            }
//            else if (thetaTarget < 0) {  //This is needed since it will otherwise behave oddly near +/- 180deg
//                if (theta <= 0) {
//                    thetaError = thetaTarget - theta;
//                }
//                else if (theta > 0) {
//                    thetaError = Math.abs(thetaTarget) - theta;
//                }
//            }

            if (Math.abs(xError) < 1) {  //Only enable integral when error is less than 1 inch
                xIntegral = xIntegral + xError*0.02;
            } else xIntegral = 0;
            if (Math.abs(yError) < 1) {  //Only enable integral when error is less than 1 inch
                yIntegral = yIntegral + yError*0.02;
            } else yIntegral = 0;
            if (Math.abs(thetaError) < 5) {  //Only enable integral when error is less than 5 degrees
                thetaIntegral = thetaIntegral + thetaError*0.02;
            } else thetaIntegral = 0;

            xDerivative = (xError - prevXError)/0.02;
            yDerivative = (yError - prevYError)/0.02;
            thetaDerivative = (thetaError - prevThetaError)/0.02;

            prevXError = xError;
            prevYError = yError;
            prevThetaError = thetaError;

            if(Math.abs(yError) < xyTol && Math.abs(xError) < xyTol && Math.abs(thetaError) < thetaTol ) {
                moveComplete = true;
            }  //If robot is within specified drive/strafe/turn tolerances, exit the loop early, otherwise it will exit after a timeout

            //PID summation
            driveCmd = Kp*xError + Ki*xIntegral + Kd*xDerivative;
            strafeCmd = Kp*yError + Ki*yIntegral + Kd*yDerivative;
            turnCmd = 0.65*(-Kp*thetaError - Ki*thetaIntegral - Kd*thetaDerivative);

            //Clip values within maximum specified power range
            driveCmd = Range.clip(driveCmd, -maxPower, maxPower);
            strafeCmd = Range.clip(strafeCmd, -maxPower, maxPower);
            turnCmd = Range.clip(turnCmd, -maxPower, maxPower);

            //Angular correction, see Wikipedia topic "Rotation Matrix"
            driveCmdtemp = driveCmd*Math.cos(Math.toRadians(theta)) - strafeCmd*Math.sin(Math.toRadians(theta));
            strafeCmdtemp = driveCmd*Math.sin(Math.toRadians(theta)) + strafeCmd*Math.cos(Math.toRadians(theta));
            driveCmd = driveCmdtemp;
            strafeCmd = strafeCmdtemp;

            //Send calculated power to wheels using mecanum equations
            motorFL.setPower(driveCmd + strafeCmd + turnCmd);
            motorFR.setPower(driveCmd - strafeCmd - turnCmd);
            motorRL.setPower(driveCmd - strafeCmd + turnCmd);
            motorRR.setPower(driveCmd + strafeCmd - turnCmd);
        }
        //Stop motors when complete
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorRL.setPower(0);
        motorRR.setPower(0);
    }
}
