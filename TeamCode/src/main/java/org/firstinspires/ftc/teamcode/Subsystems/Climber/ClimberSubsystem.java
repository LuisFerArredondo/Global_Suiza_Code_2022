package org.firstinspires.ftc.teamcode.Subsystems.Climber;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem;

public class ClimberSubsystem extends BrickSystem {
    private final DcMotorEx firstMotor;
    private final DcMotorEx secondMotor;
    private ClimberModes climberModes;


    public ClimberSubsystem(HardwareMap hardwareMap){
        firstMotor = hardwareMap.get(DcMotorEx.class, "CM1");
        secondMotor = hardwareMap.get(DcMotorEx.class, "CM2");

        firstMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        firstMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        secondMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        firstMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climberModes = ClimberModes.OFF;
    }

    public void setMotorsPower(double power){
        firstMotor.setPower(power);
        secondMotor.setPower(power);
    }

    public void setClimberModes(ClimberModes climberModes) {
        this.climberModes = climberModes;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        firstMotor.setMode(runMode);
        secondMotor.setMode(runMode);
    }
    public ClimberModes getClimberMode(){
        return climberModes;
    }

    public double getFirstMotorTicks(){
        return firstMotor.getCurrentPosition();
    }
    public double getSecondMotorTicks(){
        return secondMotor.getCurrentPosition();
    }

    public void setFirstMotorTicks(int ticks){
        firstMotor.setTargetPosition(ticks);
    }
    public void setSecondMotorTicks(int ticks){
        secondMotor.setTargetPosition(ticks);
    }
}
