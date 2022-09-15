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

public class ClimberSubsystem extends BrickSystem_V3 {
    private final DcMotorEx firstMotor;
    private final DcMotorEx secondMotor;

    private ClimberModes climberModes = ClimberModes.OFF;;
    private ClimberState actualState = ClimberState.IDLE;;



    public ClimberSubsystem(HardwareMap hardwareMap){
        firstMotor = hardwareMap.get(DcMotorEx.class, "CM");
        secondMotor = hardwareMap.get(DcMotorEx.class, "LS");

        firstMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        firstMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        secondMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        firstMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setSubsystemTelemetryColor(ColorFormatter.CYAN);
        setName("Climber Subsystem");
        setSubsystemState(actualState.toString());
    }

    public void setClimberPower(double power){
        firstMotor.setPower(power);
    }
    public void setLinearSystemPower(double power){
        secondMotor.setPower(power);
    }

    public void setActualMode(ClimberModes climberModes) {
        this.climberModes = climberModes;
    }

    public void setActualState(ClimberState climberState){
        actualState = climberState;
    }

    public ClimberState getActualState(){
        return actualState;
    }
    public ClimberModes getActualMode(){
        return climberModes;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        firstMotor.setMode(runMode);
        secondMotor.setMode(runMode);
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

    @Override
    public void periodic() {
        setSubsystemState(actualState.toString());
    }
}
