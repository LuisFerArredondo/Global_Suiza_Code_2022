package org.firstinspires.ftc.teamcode.Subsystems.Toucher;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.TouchSensorUtil;

@Config
public class ToucheSubsystem extends BrickSystem_V3 {
    //private final Servo leftPusher;
    //private final Servo rightPusher;
    public static double kP_Climber = 0;
    private DcMotorEx leftArm;
    private DcMotorEx rightArm;

    private ToucheState actualState = ToucheState.IDLE;
    private ToucheMode actualMode = ToucheMode.RETRACT;

    private final TouchSensorUtil leftSensor;
    private final TouchSensorUtil rightSensor;

    public ToucheSubsystem(HardwareMap hardwareMap){
        //leftPusher = hardwareMap.get(Servo.class, "SLP");
        //rightPusher = hardwareMap.get(Servo.class, "SRP");

        //leftPusher.setDirection(Servo.Direction.FORWARD);
        //rightPusher.setDirection(Servo.Direction.REVERSE);
        leftArm = hardwareMap.get(DcMotorEx.class, "MLA");
        rightArm = hardwareMap.get(DcMotorEx.class, "MRA");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setPositionPIDFCoefficients(kP_Climber);
        rightArm.setPositionPIDFCoefficients(kP_Climber);

        leftSensor = new TouchSensorUtil(hardwareMap, "LTS", ColorFormatter.GREEN, "TS");
        rightSensor = new TouchSensorUtil(hardwareMap, "RTS", ColorFormatter.GREEN, "TS");

        setSubsystemState(actualState.toString());
        setSubsystemTelemetryColor(ColorFormatter.LIME);
        setSubsystemSensors(leftSensor, rightSensor);

        setName("Touche Subsystem");
    }


    public ToucheState getActualState() {
        return actualState;
    }

    public void setActualState(ToucheState actualState) {
        this.actualState = actualState;
    }

    public ToucheMode getActualMode() {
        return actualMode;
    }

    public void setActualMode(ToucheMode actualMode) {
        this.actualMode = actualMode;
    }

    public void setLeftArmPower(double power){
        leftArm.setPower(power);
    }
    public void setRightArmPower(double power){
        rightArm.setPower(power);
    }

    public void setLeftArmTicks(int ticks){
        leftArm.setTargetPosition(ticks);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRightArmTicks(int ticks){
        leftArm.setTargetPosition(ticks);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getLeftCurrentPos(){
        return leftArm.getCurrentPosition();
    }
    public double getRightCurrentPos(){
        return rightArm.getCurrentPosition();
    }

    public boolean areMotorsBusy(){
        return leftArm.isBusy() && rightArm.isBusy();
    }

    public boolean areSensorsPressed(){
        return leftSensor.getButtonPressed() && rightSensor.getButtonPressed();
    }
    //public void setLeftPusherPosition(double position){
    //    leftPusher.setPosition(position);
    //}
    //public void setRightPusherPosition(double position){
    //    rightPusher.setPosition(position);
    //}

    @Override
    public void periodic() {

        setSubsystemState(getActualState().toString()
                + "\nleftArm Ticks" + leftArm.getCurrentPosition()
                + "\nrightArm Ticks" + rightArm.getCurrentPosition());
        leftSensor.changeState();
        rightSensor.changeState();
                //+ "\nleft Sensor " + leftSensor.getButtonPressed()
                //+ "\nright Sensor " + rightSensor.getButtonPressed());
    }
}
