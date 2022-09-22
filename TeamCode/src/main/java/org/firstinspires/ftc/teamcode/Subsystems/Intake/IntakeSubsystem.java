package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class IntakeSubsystem extends BrickSystem_V3 {
    private DcMotorEx intakeMotor;
    //private CRServo blender;
    private IntakeState actualState = IntakeState.IDLE;
    private IntakeMode actualMode = IntakeMode.OFF;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Itk");

        //blender =  hardwareMap.get(CRServo.class, "SB");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setName("Intake");
        setSubsystemTelemetryColor(ColorFormatter.PINK);
    }

    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }

    public IntakeState getActualState() {
        return actualState;
    }

    public void setActualState(IntakeState actualState) {
        this.actualState = actualState;
    }

    public IntakeMode getActualMode() {
        return actualMode;
    }

    public void setActualMode(IntakeMode actualMode) {
        this.actualMode = actualMode;
    }

    //public void setBlenderPower(double power){
    //    blender.setPower(power);
    //}

    public double getMotorCurrent(){
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
    @Override
    public void periodic() {
        setSubsystemState(getActualState().toString());
               // + "\nmotor current: " + getMotorCurrent());
    }
}
