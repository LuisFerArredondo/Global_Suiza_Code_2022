package org.firstinspires.ftc.teamcode.Subsystems.Toucher;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.ColorSensorUtil;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.SensorStates.ColorSensorState;

public class ToucheSubsystem extends BrickSystem_V3 {
    //private final Servo leftPusher;
    //private final Servo rightPusher;
    private Servo leftArm;
    private Servo rightArm;

    private ToucheState actualState = ToucheState.IDLE;
    private ToucheMode actualMode = ToucheMode.RETRACT;

    //private final TouchSensorUtil leftSensor;
    //private final TouchSensorUtil rightSensor;

    private final ColorSensorUtil leftSensor;
    private final ColorSensorUtil rightSensor;

    public ToucheSubsystem(HardwareMap hardwareMap) {
        //leftPusher = hardwareMap.get(Servo.class, "SLP");
        //rightPusher = hardwareMap.get(Servo.class, "SRP");

        //leftPusher.setDirection(Servo.Direction.FORWARD);
        //rightPusher.setDirection(Servo.Direction.REVERSE);
        leftArm = hardwareMap.get(Servo.class, "LS");
        rightArm = hardwareMap.get(Servo.class, "RS");

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);

        leftSensor = new ColorSensorUtil(hardwareMap, "LCS", ColorFormatter.LIME, "Left_S");
        rightSensor = new ColorSensorUtil(hardwareMap, "RCS", ColorFormatter.LIME, "Right_S");

        leftSensor.setDistanceOffset(5);
        rightSensor.setDistanceOffset(5);

        setSubsystemSensors(leftSensor, rightSensor);

        setSubsystemState(actualState.toString());
        setSubsystemTelemetryColor(ColorFormatter.LIME);

        setName("Touche_S");
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

    public void setLeftArmPosition(double position){
        leftArm.setPosition(position);
    }

    public void setRightArmPosition(double position){
        rightArm.setPosition(position);
    }

    public double getLeftSensorDistance(){
        return leftSensor.getDistance(DistanceUnit.CM);
    }
    public double getRightSensorDistance(){
        return rightSensor.getDistance(DistanceUnit.CM);
    }
    public boolean isLeftDetecting(){
        return leftSensor.getSensorState().equals(ColorSensorState.OBJECT_DETECTED.toString());
    }
    public boolean isRightDetecting(){
        return rightSensor.getSensorState().equals(ColorSensorState.OBJECT_DETECTED.toString());
    }

    //public void setLeftPusherPosition(double position){
    //    leftPusher.setPosition(position);
    //}
    //public void setRightPusherPosition(double position){
    //    rightPusher.setPosition(position);
    //}

    @Override
    public void periodic() {

        setSubsystemState(getActualState().toString());
                //+ "left Sensor Distance: " + getLeftSensorDistance()
                //+ "right Sensor Distance: " + getRightSensorDistance());
                //+ "\nleftArm Ticks" + leftArm.getCurrentPosition()
                //+ "\nrightArm Ticks" + rightArm.getCurrentPosition());

        leftSensor.changeState();
        rightSensor.changeState();
                //+ "\nleft Sensor " + leftSensor.getButtonPressed()
                //+ "\nright Sensor " + rightSensor.getButtonPressed());
    }
}
