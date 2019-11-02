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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Hardwarebot Basic Driver", group="Iterative Opmode")
@Disabled
public class Hardwarebot_Basic_Driver_Op extends OpMode
{
    //Drive system declarations
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorRL = null;
    private DcMotor motorRR = null;
    private double driver1SpeedKTurbo = 1.0;  //1.0 = 100% power
    private double driver1SpeedKStandard = 0.50;  //0.XX = XX% power
    private double driver1SpeedKLast = driver1SpeedKTurbo;  //Initialize at turbo power
    private double driver1SpeedKTemp = driver1SpeedKTurbo;  //Initialize at turbo power
    private double driver1SpeedKFinal = driver1SpeedKTurbo;  //Initialize at turbo power
    private double motorFLpower1 = 0;
    private double motorFRpower1 = 0;
    private double motorRLpower1 = 0;
    private double motorRRpower1 = 0;
    private double motorFLpowerFinal = 0;
    private double motorFRpowerFinal = 0;
    private double motorRLpowerFinal = 0;
    private double motorRRpowerFinal = 0;
    //Intake/outtake system declarations
    private DcMotor intakeR = null;
    private DcMotor intakeL = null;
    private double intakeOpStop = 0.00;  //0%
    private double intakeOpStart = 1.0;  //100%
    private double intakeOpLast = intakeOpStop;  //Initialize at stop
    private double intakeOpTemp = intakeOpStop;  //Initialize at stop
    private double intakeOpFinal = intakeOpStop;  //Initialize at stop
    private double outtakeOpStop = 0.00;  //0%
    private double outtakeOpStart = -1.0;  //-100%
    private double outtakeOpLast = outtakeOpStop;  //Initialize at stop
    private double outtakeOpTemp = outtakeOpStop;  //Initialize at stop
    private double outtakeOpFinal = outtakeOpStop;  //Initialize at stop


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
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorRL.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


//*******DRIVE OPERATION**************************************************************************//
        // Left stick to go forward and strafe, and right stick to turn.
        double drive1 = -gamepad1.left_stick_y;  //Driver 1 drive command
        double strafe1 = gamepad1.left_stick_x;  //Driver 1 strafe command
        double turn1 = gamepad1.right_stick_x;  //Driver 1 turn command

        // Toggle driver1 speed (turbo or standard) when pressing Start button
        if (gamepad1.start && driver1SpeedKLast < driver1SpeedKTurbo) {
            driver1SpeedKTemp = driver1SpeedKTurbo;
        } else if (gamepad1.start && driver1SpeedKLast > driver1SpeedKStandard) {
            driver1SpeedKTemp = driver1SpeedKStandard;
        }
        if (!gamepad1.start && driver1SpeedKLast != driver1SpeedKTemp) {  //This is to prevent toggle bounce when holding the Start button; e.g. toggle on release
            driver1SpeedKLast = driver1SpeedKTemp;
        }
        driver1SpeedKFinal = driver1SpeedKTemp;  //Driver 1 speed gain

        // Driver 1 motor power using mecanum equations
        motorFLpower1 = driver1SpeedKFinal*(drive1 + turn1 + strafe1);
        motorFRpower1 = driver1SpeedKFinal*(drive1 - turn1 - strafe1);
        motorRLpower1 = driver1SpeedKFinal*(drive1 + turn1 - strafe1);
        motorRRpower1 = driver1SpeedKFinal*(drive1 - turn1 + strafe1);

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

        telemetry.addData("Kdriver1", driver1SpeedKFinal);
//************************************************************************************************//


//*******INTAKE/OUTTAKE OPERATION*****************************************************************//
        // Toggle Intake when pressing Left Bumper
        if (gamepad2.left_bumper && intakeOpLast < intakeOpStart) {
            intakeOpTemp = intakeOpStart;
        } else if (gamepad2.left_bumper && intakeOpLast > intakeOpStop) {
            intakeOpTemp = intakeOpStop;
        }
        if (!gamepad2.left_bumper && intakeOpLast != intakeOpTemp) {  //This is to prevent toggle bounce when holding the Left Bumper; e.g. toggle on release
            intakeOpLast = intakeOpTemp;
            //Turn off outtake if it is already on
            outtakeOpFinal = 0.00;
            outtakeOpTemp = 0.00;
            outtakeOpLast = 0.00;
        }
        intakeOpFinal = intakeOpTemp;  //intake Motor speed

        // Toggle Outtake when pressing Right Bumper
        if (gamepad2.right_bumper && outtakeOpLast < outtakeOpStart) {
            outtakeOpTemp = outtakeOpStart;
        } else if (gamepad2.right_bumper && outtakeOpLast > outtakeOpStop) {
            outtakeOpTemp = outtakeOpStop;
        }
        if (!gamepad2.right_bumper && outtakeOpLast != outtakeOpTemp) {  //This is to prevent toggle bounce when holding the Right Bumper; e.g. toggle on release
            outtakeOpLast = outtakeOpTemp;
            //Turn off intake if it is already on
            intakeOpFinal = 0.00;
            intakeOpTemp = 0.00;
            intakeOpLast = 0.00;
        }
        outtakeOpFinal = outtakeOpTemp;  //outtake Motor speed

        // Send calculated power to intake/outtake Motors
        intakeR.setPower(intakeOpFinal + outtakeOpFinal);  //_Both_ intake or outtake should _not_ be true at the same time
        intakeL.setPower(intakeOpFinal + outtakeOpFinal);  //_Both_ intake or outtake should _not_ be true at the same time
//************************************************************************************************//


        // Add telemetry
//        telemetry.addData("Status", "Run Time: " + runtime.toString());

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
