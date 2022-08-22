package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem;

public class TestSubsystem extends BrickSystem {

    private DcMotorEx shooter;

    public TestSubsystem(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "SM");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(1,0,0,14);
    }

    public int getPortNumber(){
        return shooter.getPortNumber();
    }

    public String getConnectionInfo(){
        return shooter.getConnectionInfo();
    }

    public double getCurrent() {
        return shooter.getCurrent(CurrentUnit.MILLIAMPS);
    }
}

