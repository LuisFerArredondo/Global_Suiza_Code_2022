package org.firstinspires.ftc.teamcode.Team15600Lib.Util;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class used to indicate {@link CommandScheduler} which class is a subsystem
 **/
public abstract class BrickSystem_V3 implements Subsystem {

    protected String m_name = this.getClass().getSimpleName();
    private String m_state;
    private ColorFormatter m_color = ColorFormatter.WHITE;
    private List<BrickSensor> m_sensors = new ArrayList<>();

    public BrickSystem_V3() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public String getName() {
        return m_name;
    }

    public void setName(String name) {
        m_name = name;
    }

    public String getSubsystem() {
        return getName();
    }

    public void setSubsystem(String subsystem) {
        setName(subsystem);
    }

    public String getSubsystemState() {
        return m_state;
    }

    public void setSubsystemState(String state){
        m_state = state;
    }

    public void setSubsystemTelemetryColor(ColorFormatter color){
        m_color = color;
    }

    public ColorFormatter getSelectedTelemetryColor(){
        return m_color;
    }

    public void sendFtcDashboardTelemetryPacket(TelemetryPacket packet){}
    public void sendFtcDashboardFieldOverlay(Canvas fieldOverlay){}

    public void setSubsystemSensors(BrickSensor... sensorsState){
        m_sensors.addAll(Arrays.asList(sensorsState));
    }
    public List<BrickSensor> getSubsystemSensors(){
        return m_sensors;
    }
}


