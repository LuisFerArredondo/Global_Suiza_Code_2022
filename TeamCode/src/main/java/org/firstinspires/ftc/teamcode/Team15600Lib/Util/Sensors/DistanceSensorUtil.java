package org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.SensorStates.DistanceSensorState;

public class DistanceSensorUtil extends BrickSensor {
    private DistanceSensor distanceSensor;
    private DistanceSensorState sensorState = DistanceSensorState.IS_OFF;
    private double offset = 78;
    private boolean isInitialized = false;

    public DistanceSensorUtil(HardwareMap hardwareMap, String name) {
        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, name);
            isInitialized = true;
            sensorState = DistanceSensorState.IS_OUT_OF_RANGE;
        } catch (Exception ex) {
            isInitialized = false;
            sensorState = DistanceSensorState.IS_OFF;
            RobotLog.dd("Distance Sensor \"" + name + "\" State: ", "not Found, error msg: " + ex);
        }
        setActualState(sensorState.toString());
    }

    public DistanceSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel) {
        this(hardwareMap, name);
        setSensorTelemetryColor(colorForTel);
    }

    public DistanceSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel, String telemetryName) {
        this(hardwareMap, name, colorForTel);
        setSensorTelemetryColor(colorForTel);
        setName(telemetryName);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return isInitialized ? distanceSensor.getDistance(distanceUnit) : 0;
    }

    public void setOffset(double offset){
        this.offset = offset;
    }

    public void changeState() {

        if (getDistance(DistanceUnit.INCH) > 0 && getDistance(DistanceUnit.INCH) < offset)
            sensorState = DistanceSensorState.IS_IN_RANGE;
        else sensorState = DistanceSensorState.IS_OUT_OF_RANGE;

        if(!isInitialized) sensorState = DistanceSensorState.IS_OFF;

        setActualState(sensorState.toString());
    }
}
