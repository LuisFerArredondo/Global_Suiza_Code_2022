package org.firstinspires.ftc.teamcode.Subsystems.ClimberLock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private static LockStates actualState = LockStates.IS_OPENED;


    public ClimberLockSubsystem(HardwareMap hardwareMap){
        lock = hardwareMap.get(Servo.class, "SL");

        lock.setDirection(Servo.Direction.REVERSE);
        //controllerEx = hardwareMap.get(ServoControllerEx.class, "SL");
        //controllerEx = lock.getController();

        setSubsystemState(actualState.toString());
        setSubsystemTelemetryColor(ColorFormatter.LIME);
        setName("Locker_S");
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

    public static LockStates getActualState() {
        return actualState;
    }

    public void setActualState(LockStates actualState) {
        this.actualState = actualState;
    }

    @Override
    public void periodic() {
        setSubsystemState(getActualState().toString());
                //+ "Servo Position based on Controller" + controllerEx.getServoPosition(lock.getPortNumber()));
    }
}
