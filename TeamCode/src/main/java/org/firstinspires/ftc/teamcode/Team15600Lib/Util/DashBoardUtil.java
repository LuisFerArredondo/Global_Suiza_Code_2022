package org.firstinspires.ftc.teamcode.Team15600Lib.Util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class DashBoardUtil implements Runnable{
    private static DashBoardUtil instance;
    private TelemetryPacket telemetryPacket;
    private Canvas fieldOverlay;

    public static synchronized DashBoardUtil getInstance() {
        if (instance == null) {
            instance = new DashBoardUtil();
        }
        return instance;
    }

    DashBoardUtil(){
        telemetryPacket = new TelemetryPacket();
        fieldOverlay = telemetryPacket.fieldOverlay();
    }

    public void sendPacket(String key, Object obj){
        telemetryPacket.put(key, obj);
    }
    public void sendStrokeLine(String color,double x1,double y1,double x2,double y2){
        fieldOverlay.setStroke(color);
        fieldOverlay.strokeLine(x1, y1, x2, y2);
    }

    public void sendStrokeCircle(String color, double x, double y, double radius){
        fieldOverlay.setStroke(color);
        fieldOverlay.strokeCircle(x, y, radius);
    }

    public void sendStrokeRectangle(String color, double x, double y, double width, double height){
        fieldOverlay.setStroke(color);
        fieldOverlay.strokeRect(x, y, width, height);
    }

    @Override
    public void run() {

    }
}
