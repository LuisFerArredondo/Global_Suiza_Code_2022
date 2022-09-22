package org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.SensorStates.TouchSensorState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

public class TouchSensorUtil extends BrickSensor{
    private final TouchSensor touchSensor;
    private TouchSensorState sensorState = TouchSensorState.IS_RELEASED;

    public TouchSensorUtil(HardwareMap hardwareMap, String name) {
        this.touchSensor = hardwareMap.get(TouchSensor.class, name);
    }
    public TouchSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel) {
        this(hardwareMap, name);
        setSensorTelemetryColor(colorForTel);
        setActualState(sensorState.toString());
    }
    public TouchSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel, String telemetryName) {
        this(hardwareMap, name, colorForTel);
        setSensorTelemetryColor(colorForTel);
        setName(telemetryName);
        setActualState(sensorState.toString());
    }

    public boolean getButtonPressed(){
        return touchSensor.isPressed();
    }

    public void changeState(){
        if(touchSensor.isPressed())
            sensorState = TouchSensorState.IS_PRESSED;
        else
            sensorState = TouchSensorState.IS_RELEASED;

        setActualState(sensorState.toString());
    }
}
