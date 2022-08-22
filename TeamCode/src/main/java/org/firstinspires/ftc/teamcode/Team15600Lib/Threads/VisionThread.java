package org.firstinspires.ftc.teamcode.Team15600Lib.Threads;

import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.VisionState;

public abstract class VisionThread extends Thread{
    public abstract VisionState getVisionThreadState();
}
