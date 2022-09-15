package org.firstinspires.ftc.teamcode.Subsystems.ClimberLock;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class ClimberLockSubsystem extends BrickSystem_V3 {
    private Servo lock;
    private LockMode actualMode = LockMode.OPEN;
    private LockStates actualState = LockStates.IS_OPENED;

    private ServoController controllerEx;

    public ClimberLockSubsystem(HardwareMap hardwareMap){
        lock = hardwareMap.get(Servo.class, "SL");

        //controllerEx = hardwareMap.get(ServoControllerEx.class, "SL");
        controllerEx = lock.getController();

        setSubsystemState(actualState.toString());
        setSubsystemTelemetryColor(ColorFormatter.ORANGE);
        setName("Lockers");
    }

    public void setLockPosition(double position){
        lock.setPosition(position);
    }

    public LockMode getActualMode() {
        return actualMode;
    }

    public void setActualMode(LockMode actualMode) {
        this.actualMode = actualMode;
    }

    public LockStates getActualState() {
        return actualState;
    }

    public void setActualState(LockStates actualState) {
        this.actualState = actualState;
    }

    @Override
    public void periodic() {
        setSubsystemState(getActualState().toString()
                + "Servo Position based on Controller" + controllerEx.getServoPosition(lock.getPortNumber()));
    }
}
