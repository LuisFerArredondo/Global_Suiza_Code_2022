package org.firstinspires.ftc.teamcode.Subsystems.LinearSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemStates;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class LinearSystemSubsystem extends BrickSystem_V3 {
    private final DcMotorEx linearSystem;
    private LinearSystemModes actualMode = LinearSystemModes.OFF;
    private LinearSystemStates actualState = LinearSystemStates.IDLE;

    public LinearSystemSubsystem(HardwareMap hardwareMap) {
        linearSystem = hardwareMap.get(DcMotorEx.class, "LSM");
        linearSystem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSystem.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSystem.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSystem.setTargetPositionTolerance(60);
        linearSystem.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSystem.setPositionPIDFCoefficients(4);
        setSubsystemTelemetryColor(ColorFormatter.CYAN);
        setName("LinearSystem Subsystem");
        setSubsystemState(actualState.toString());
    }

    public void setLinearSystemPower(double power){
        //secondMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSystem.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode){
        linearSystem.setMode(runMode);
    }
    public double getLinearSystemMotorTicks(){
        return linearSystem.getCurrentPosition();
    }

    public void setLinearSystemMotorTicks(int ticks){
        linearSystem.setTargetPosition(ticks);
        linearSystem.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getLinearSystemCurrent(){
        return linearSystem.getCurrent(CurrentUnit.AMPS);
    }

    public LinearSystemModes getActualMode() {
        return actualMode;
    }

    public void setActualMode(LinearSystemModes actualMode) {
        this.actualMode = actualMode;
    }
    public LinearSystemStates getActualState() {
        return actualState;
    }

    public void setActualState(LinearSystemStates actualState) {
        this.actualState = actualState;
    }

    public boolean isMotorBusy(){
        return linearSystem.isBusy();
    }

    @Override
    public void periodic() {
        setSubsystemState(actualState.toString());
                //+ "\nIs Motor Busy: " + isMotorBusy()
                //+ "\nlinearSystem Ticks: " + getLinearSystemMotorTicks());
    }
}
