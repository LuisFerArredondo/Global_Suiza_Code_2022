package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * Class used to indicate {@link CommandScheduler} which class is a subsystem
 * */
public abstract class BrickSystem implements Subsystem {

    protected String m_name = this.getClass().getSimpleName();

    public BrickSystem() {
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

}