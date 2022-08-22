package org.firstinspires.ftc.teamcode.Team15600Lib.Util;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Class used to indicate {@link CommandScheduler} which class is a subsystem
 **/
public abstract class BrickSystem_V2 implements Subsystem {

    protected String m_name = this.getClass().getSimpleName();
    private String m_state = setSubsystemState();
    private ColorFormatter m_color = ColorFormatter.WHITE;

    public BrickSystem_V2() {
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

    public abstract String setSubsystemState();

    public void setSubsystemTelemetryColor(ColorFormatter color){
        m_color = color;
    }

    public ColorFormatter getSelectedTelemetryColor(){
        return m_color;
    }
}


