package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Codebusters in 2019.
 */
class pidDriveCalculator{

    //Position declaration
    double xCurrent, yCurrent, thetaCurrent;  //[inch]
    double xTarget, yTarget, thetaTarget;  //[inch]

    //PID controller declarations
    double Kp = 0.1;  //[--]
    double Ki = 0;  //[--]
    double Kd = 0;  //[--]
    double xError;  //[in]
    double yError;  //[in]
    double thetaError;  //[deg]

    //Output declarations
    double maxPower;  //[%]; Maximum output value
    double driveCmd = 0;  //[%]; Drive command = 0 - 1.00
    double strafeCmd = 0;  //[%]; Strafe command = 0 - 1.00
    double turnCmd = 0;  //[%]; Turn command = 0 - 1.00

    /**
     * Import values and assign locally.  Input units must be as specified
     */
    public void pidDriveCalculator(double absPosnX, double absPosnY, double absPosnTheta, double xTarget, double yTarget, double thetaTarget, double maxPower){
        this.xCurrent = absPosnX;  //[inch];  Current x position
        this.yCurrent = absPosnY;  //[inch];  Current y position
        this.thetaCurrent = absPosnTheta;  //[deg];  Current heading
        this.xTarget = xTarget;  //[inch];  Target x position
        this.yTarget = yTarget;  //[inch];  Target y position
        this.thetaTarget = thetaTarget;  //[deg];  Target heading
        this.maxPower = maxPower;  //[%];  Maximum output power

        //Calculate error
        xError = xTarget - absPosnX;
        yError = yTarget - absPosnX;
        thetaError = thetaTarget - absPosnX;

        //PID summation
        driveCmd = Kp * yError;
        strafeCmd = Kp * xError;
        turnCmd = Kp * thetaError;

        //Clip values within maximum specified power range
        driveCmd = Range.clip(driveCmd, -maxPower, maxPower);
        strafeCmd = Range.clip(strafeCmd, -maxPower, maxPower);
        turnCmd = Range.clip(turnCmd, -maxPower, maxPower);
//        return(driveCmd);
    }


    /**
     * Returns the calculated drive command
     */
    public double returnDriveCmd(){
        return driveCmd;  //[%];  Output ranges from 0 - 1.00
    }

    /**
     * Returns the calculated strafe command
     */
    public double returnStrafeCmd(){
        return strafeCmd;  //[%];  Output ranges from 0 - 1.00
    }

    /**
     * Returns the calculated turn command
     */
    public double returnTurnCmd(){
        return turnCmd;  //[%];  Output ranges from 0 - 1.00
    }
}
