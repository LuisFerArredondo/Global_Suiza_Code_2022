package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorUtil {
    private DistanceSensor distanceSensor;
    private boolean isSensorFound = false;

    public DistanceSensorUtil(HardwareMap hardwareMap, String name) {
        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, name);
            isSensorFound = true;
        } catch (Exception ex) {
            isSensorFound = false;
            RobotLog.dd("Distance Sensor \"" + name + "\" State: ", "not Found, error msg: " + ex);
        }
    }

    public double getDistance(DistanceUnit distanceUnit){
        return isSensorFound ? distanceSensor.getDistance(distanceUnit) : 0;
    }
}
