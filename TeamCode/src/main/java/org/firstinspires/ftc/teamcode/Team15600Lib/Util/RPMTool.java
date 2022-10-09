package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * RPMTool can read and write RPM values
 *
 */

public class RPMTool {

    public double TICKS_PER_REVOLUTION = 0;

    private ElapsedTime time;

    private DcMotor motor;

    private double Time = 0.001; // set a non 0 value to prevent any initial div by 0
    private double ticks = 0;

    private double lastTicks = 0;
    private double lastTime = 0;

    /*
     * motor that you want to read the RPM of or write the RPM needs to be passed as the motor parameter.
     * The ticks per revolution also needs to be passed as a parameter (you'll find value on motor website).
     */
    public RPMTool(DcMotor motor, double TICKS_PER_REVOLUTION){

        this.TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;

        this.motor = motor;

        time = new ElapsedTime();

        time.reset();
    }


    // keep track ticks per sec
    public double ticksPerSec(){

        ticks = motor.getCurrentPosition() - lastTicks;
        Time = time.seconds() - lastTime;

        double tickVelocity = ticks / Time;

        if (Time > 1){
            lastTicks = motor.getCurrentPosition();
            lastTime = time.seconds();
        }

        return tickVelocity;
    }

    // multiply by 60 then divide ticks vel by total ticks in one rotation to get the rpm
    public double getRPM(){
        double ticks = ticksPerSec() * 60;

        double RPM = ticks / TICKS_PER_REVOLUTION;

        RPM = Math.ceil(RPM); // we will round up

        return RPM;
    }

    // convert RPM to ticks per sec then set as velocity
    public void setRPM(double targetRPM){
        //motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // I only have to set this once

        // convert rpm to ticks per sec
        double ticksPerSec = targetRPM * TICKS_PER_REVOLUTION / 60;

        // set velocity
        //motor.setVelocity(ticksPerSec);
    }

}