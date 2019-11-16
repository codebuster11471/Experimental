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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Softwarebot Basic Driver with IMU", group="Iterative Opmode")
@Disabled
public class Softwarebot_Basic_Driver_Op_with_IMU extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double driver1SpeedKTurbo = 1.0;  //1.0 = 100% power
    private double driver1SpeedKStandard = 0.25;  //0.XX = XX% power
    private double driver1SpeedKLast = driver1SpeedKStandard;  //Initialize at standard power
    private double driver1SpeedKTemp = driver1SpeedKStandard;  //Initialize at standard power
    private double driveSpeedKFinal = driver1SpeedKStandard;  //Initialize at standard power
    private double motorFLpower1 = 0;
    private double motorFRpower1 = 0;
    private double motorRLpower1 = 0;
    private double motorRRpower1 = 0;
    private double motorFLpowerFinal = 0;
    private double motorFRpowerFinal = 0;
    private double motorRLpowerFinal = 0;
    private double motorRRpowerFinal = 0;
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorRL = null;
    private DcMotor motorRR = null;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorRL = hardwareMap.get(DcMotor.class, "motorRL");
        motorRR = hardwareMap.get(DcMotor.class, "motorRR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);


        // Get a reference to a Modern Robotics gyro object.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.

        modernRoboticsI2cGyro.calibrate();
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        telemetry.update();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (modernRoboticsI2cGyro.isCalibrating())  {
        //Wait
        }

        // Tell the driver that initialization is complete.
        telemetry.log().clear();
        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.log().clear();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Read heading data from IMU
        double heading = modernRoboticsI2cGyro.getHeading();

        //        telemetry.addData("Status", "Run Time: " + runtime.toString());


        telemetry.addData("Heading", "%3.0f deg", heading);

        // Left stick to go forward and strafe, and right stick to turn.
        double leftDrive1 = -gamepad1.left_stick_y;
        double leftStrafe1 = gamepad1.left_stick_x;
        double rightTurn1 = gamepad1.right_stick_x;

        // Toggle driver1 speed (turbo or standard) when pressing Start button
        if(gamepad1.start && driver1SpeedKLast < driver1SpeedKTurbo){
            driver1SpeedKTemp = driver1SpeedKTurbo;
        }else if (gamepad1.start && driver1SpeedKLast > driver1SpeedKStandard) {
            driver1SpeedKTemp = driver1SpeedKStandard;
        }
        if(!gamepad1.start && driver1SpeedKLast != driver1SpeedKTemp){  //This is to prevent toggle bounce when holding the Start button; e.g. toggle on release
            driver1SpeedKLast = driver1SpeedKTemp;
        }
        driveSpeedKFinal = driver1SpeedKTemp;  //Driver 1 speed gain


        //IMU heading correction.  Apply rotation to drive and strafe vectors
        double[][] rotationMatrix = { {Math.cos(Math.toRadians(heading)), -Math.sin(Math.toRadians(heading))},
                {Math.sin(Math.toRadians(heading)), Math.cos(Math.toRadians(heading))}  };
        double[][] inputArrayTemp = { {leftDrive1}, {leftStrafe1} };

        double[][] outputArray = new double [2][1];

        outputArray[0][0] = rotationMatrix[0][0]*inputArrayTemp[0][0] + rotationMatrix[0][1]*inputArrayTemp[1][0];
        outputArray[1][0] = rotationMatrix[1][0]*inputArrayTemp[0][0] + rotationMatrix[1][1]*inputArrayTemp[1][0];

        leftDrive1 = outputArray[0][0];
        leftStrafe1 = outputArray[1][0];

        telemetry.addData("newDrive", outputArray[0][0]);
        telemetry.addData("newStrafe", outputArray[1][0]);


        // Driver 1 motor power using mecanum equations
        motorFLpower1 = driveSpeedKFinal*(leftDrive1 + rightTurn1 + leftStrafe1);
        motorFRpower1 = driveSpeedKFinal*(leftDrive1 - rightTurn1 - leftStrafe1);
        motorRLpower1 = driveSpeedKFinal*(leftDrive1 + rightTurn1 - leftStrafe1);
        motorRRpower1 = driveSpeedKFinal*(leftDrive1 - rightTurn1 + leftStrafe1);

        // Add up all motor power sources
        motorFLpowerFinal = motorFLpower1;
        motorFRpowerFinal = motorFRpower1;
        motorRLpowerFinal = motorRLpower1;
        motorRRpowerFinal = motorRRpower1;

        // Send calculated power to wheels using mecanum equations
        motorFL.setPower(motorFLpowerFinal);
        motorFR.setPower(motorFRpowerFinal);
        motorRL.setPower(motorRLpowerFinal);
        motorRR.setPower(motorRRpowerFinal);



        // Add telemetry
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Kdriver1", driveSpeedKFinal);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addLine("kthxbye");
    }


}
