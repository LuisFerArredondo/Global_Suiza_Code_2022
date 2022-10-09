package org.firstinspires.ftc.teamcode.Subsystems.Climber;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class ClimberSubsystem extends BrickSystem_V3 {
    private final DcMotorEx internalClimber;
    private final DcMotorEx externalClimber;

    private ClimberModes climberModes = ClimberModes.OFF;
    private ClimberState actualState = ClimberState.IDLE;

    public ClimberSubsystem(HardwareMap hardwareMap) {
        internalClimber = hardwareMap.get(DcMotorEx.class, "ICM");
        externalClimber = hardwareMap.get(DcMotorEx.class, "ECM");

        internalClimber.setMode(RunMode.STOP_AND_RESET_ENCODER);
        externalClimber.setMode(RunMode.STOP_AND_RESET_ENCODER);

        internalClimber.setMode(RunMode.RUN_WITHOUT_ENCODER);
        externalClimber.setMode(RunMode.RUN_WITHOUT_ENCODER);

        internalClimber.setDirection(DcMotorSimple.Direction.FORWARD);
        externalClimber.setDirection(DcMotorSimple.Direction.FORWARD);

        internalClimber.setTargetPositionTolerance(100);
        externalClimber.setTargetPositionTolerance(100);

        internalClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        externalClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Estos weyes se estan juganod al verga y me da cucu lo que le pase al robox

        setSubsystemTelemetryColor(ColorFormatter.CYAN);
        setName("Climber Subsystem");
        setSubsystemState(actualState.toString());
    }

    //power
    public void setClimberPower(double power) {
        //firstMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        internalClimber.setPower(power);
        externalClimber.setPower(power);
    }

    //Managers
    public void setActualMode(ClimberModes climberModes) {
        this.climberModes = climberModes;
    }

    public void setActualState(ClimberState climberState) {
        actualState = climberState;
    }

    public ClimberState getActualState() {
        return actualState;
    }

    public ClimberModes getActualMode() {
        return climberModes;
    }

    //run Mode
    public void setRunMode(RunMode runMode) {
        internalClimber.setMode(runMode);
        externalClimber.setMode(runMode);
    }

    //Ticks
    public double getClimberMotorTicks() {
        return (double) (internalClimber.getCurrentPosition()
                + externalClimber.getCurrentPosition()) / 2;
    }

    public void setClimberMotorTicks(int ticks) {
        internalClimber.setTargetPosition(ticks);
        externalClimber.setTargetPosition(ticks);
        internalClimber.setMode(RunMode.RUN_TO_POSITION);
        externalClimber.setMode(RunMode.RUN_TO_POSITION);
    }

    public double getClimberCurrent() {
        return internalClimber.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isMotorBusy() {
        return internalClimber.isBusy();
    }

    @Override
    public void periodic() {
        setSubsystemState(actualState.toString());
        //+ "\nLinearSystem  : " + getLinearSystemMotorTicks()
        //+ "\nClimberSystem  : " + getClimberMotorTicks());
    }
}
