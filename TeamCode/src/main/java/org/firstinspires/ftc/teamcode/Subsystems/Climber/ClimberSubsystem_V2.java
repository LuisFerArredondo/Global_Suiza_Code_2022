package org.firstinspires.ftc.teamcode.Subsystems.Climber;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class ClimberSubsystem_V2 extends BrickSystem_V3 {
    private DcMotorEx climberArm;
    private DcMotorEx climberClimber;

    private Servo climberStabilizer;

    private ClimberModes climberModes = ClimberModes.OFF;
    private ClimberState climberState = ClimberState.IDLE;

    public ClimberSubsystem_V2(HardwareMap hardwareMap){
        climberArm = hardwareMap.get(DcMotorEx.class, "CA");
        climberClimber = hardwareMap.get(DcMotorEx.class, "CC");

        climberStabilizer = hardwareMap.get(Servo.class, "CS");

        climberArm.setDirection(DcMotorSimple.Direction.FORWARD);
        climberClimber.setDirection(DcMotorSimple.Direction.FORWARD);

        climberArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberClimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        climberArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberClimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        climberArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setName("Climber");
        setSubsystemTelemetryColor(ColorFormatter.LIME);
        setSubsystemState(getActualState().toString());
    }

    public void setClimberArmMode(DcMotor.RunMode runMode){
        climberArm.setMode(runMode);
    }
    public void setClimberClimberMode(DcMotor.RunMode runMode){
        climberClimber.setMode(runMode);
    }

    public void setClimberArmPower(double power){
        climberArm.setPower(power);
    }
    public void setClimberClimberPower(double power){
        climberClimber.setPower(power);
    }
    public void setClimberArmTicks(int ticks){
        climberClimber.setTargetPosition(ticks);
    }
    public void setClimberClimberTicks(int ticks){
        climberClimber.setTargetPosition(ticks);
    }

    public void setClimberStabilizerPos(double position){
        climberStabilizer.setPosition(position);
    }

    public void setActualMode(ClimberModes climberModes){
        this.climberModes = climberModes;
    }

    public void setActualState(ClimberState climberState){
        this.climberState = climberState;
    }

    public ClimberModes getActualMode(){
        return climberModes;
    }
    public ClimberState getActualState(){
        return climberState;
    }

    public double getClimberArmTicks(){
        return climberArm.getCurrentPosition();
    }
    public double getClimberClimberTicks(){
        return climberClimber.getCurrentPosition();
    }

    @Override
    public void periodic() {
        setSubsystemState(getActualState().toString());
                //+ "\nClimber Ticks" + getClimberArmTicks());
    }
}
