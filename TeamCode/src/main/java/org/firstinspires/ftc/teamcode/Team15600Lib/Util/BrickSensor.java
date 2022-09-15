package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

public class BrickSensor {
    private String m_state = "NO_STATE";
    private ColorFormatter m_color = ColorFormatter.WHITE;
    private String m_name = this.getClass().getSimpleName();

    public String getSensorState() {
        return m_state;
    }

    protected void setActualState(String m_state) {
        this.m_state = m_state;
    }

    protected void setSensorTelemetryColor(ColorFormatter color){
        m_color = color;
    }

    public ColorFormatter getSelectedTelemetryColor(){
        return m_color;
    }

    protected void setName(String name){
        m_name = name;
    }

    public String getName(){
        return m_name;
    }
}
