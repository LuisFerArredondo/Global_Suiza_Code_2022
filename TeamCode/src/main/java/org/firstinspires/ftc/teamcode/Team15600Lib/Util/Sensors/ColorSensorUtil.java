package org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.CodeObserver;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.SensorStates.ColorSensorState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.SensorStates.Colors;

public class ColorSensorUtil extends BrickSensor {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private ColorSensorState sensorState = ColorSensorState.IS_OFF;
    private double distanceOffset = 9;
    private boolean isInitialized = false;

    public ColorSensorUtil(HardwareMap hardwareMap, String name) {
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, name);
            distanceSensor = hardwareMap.get(DistanceSensor.class, name);
            isInitialized = true;

        } catch (Exception ex) {
            RobotLog.dd("Color Sensor \"" + name + "\" State: ", "not Found, error msg: " + ex);
            isInitialized = false;
        }
        setActualState(sensorState.toString());
        lightOn();
    }

    public ColorSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel) {
        this(hardwareMap, name);
        setSensorTelemetryColor(colorForTel);
    }

    public ColorSensorUtil(HardwareMap hardwareMap, String name, ColorFormatter colorForTel, String telemetryName) {
        this(hardwareMap, name, colorForTel);
        setSensorTelemetryColor(colorForTel);
        setName(telemetryName);
    }

    public void lightOn() {
        if (isInitialized) {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
        }
    }

    public void lightOff() {
        if (isInitialized) {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
        }
    }

    public double getColor(Colors color) {
        double getColor = 0;

        if (isInitialized) {
            float hsvValues[] = {0F, 0F, 0F};
            final float values[] = hsvValues;
            final double SCALE_FACTOR = 255;
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            switch (color) {
                case RED:
                    getColor = colorSensor.red();
                    break;
                case GREEN:
                    getColor = colorSensor.green();
                    break;
                case BLUE:
                    getColor = colorSensor.blue();
                    break;
                default:
                    getColor = 0;
                    break;
            }
        }
        return getColor;
    }

    public double getDistance(DistanceUnit distanceUnit) {
        double getDistance = 0;

        if (isInitialized) {
                getDistance = distanceSensor.getDistance(distanceUnit);
        }
        return getDistance;
    }

    public void changeState() {
        if (getDistance(DistanceUnit.CM) < distanceOffset && getDistance(DistanceUnit.CM) > 0)
            sensorState = ColorSensorState.OBJECT_DETECTED;
        else sensorState = ColorSensorState.IS_OUT_OF_RANGE;

        if (!isInitialized) sensorState = ColorSensorState.IS_OFF;

        setActualState(sensorState.toString());
    }

    public void setDistanceOffset(double distanceOffset) {
        this.distanceOffset = distanceOffset;
    }
}
