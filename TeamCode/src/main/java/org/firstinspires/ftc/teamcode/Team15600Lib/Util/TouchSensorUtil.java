package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.TouchSensorState;

public class TouchSensorUtil extends BrickSensor{
    private TouchSensor touchSensor;
    private TouchSensorState actualState = TouchSensorState.IS_RELEASED;

    public TouchSensorUtil(HardwareMap hardwareMap, String name) {
        this.touchSensor = hardwareMap.get(TouchSensor.class, name);
    }
    public TouchSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel) {
        this(hardwareMap, name);
        setSensorTelemetryColor(colorForTel);
        setActualState(actualState.toString());
    }
    public TouchSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel, String telemetryName) {
        this(hardwareMap, name, colorForTel);
        setSensorTelemetryColor(colorForTel);
        setName(telemetryName);
        setActualState(actualState.toString());
    }

    public boolean getButtonPressed(){
        return touchSensor.isPressed();
    }

    public TouchSensorState getActualSensorState() {
        return actualState;
    }

    public void setActualSensorState(TouchSensorState sensorState) {
        this.actualState = sensorState;
    }

    public void changeState(){
        if(touchSensor.isPressed())
            actualState = TouchSensorState.IS_PRESSED;
        else
            actualState = TouchSensorState.IS_RELEASED;

        setActualState(actualState.toString());
    }
}
