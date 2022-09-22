package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.arcrobotics.ftclib.util.Timing.Timer;

import java.util.concurrent.TimeUnit;

public class JustOnce {
    private boolean flag = false;
    private Timer timer;

    public JustOnce(){}


    public void JustOneTime(boolean trigger, Runnable toRun){
        if(trigger && !flag){
            toRun.run();
            flag = true;
        }
        if(!trigger) flag = false;
    }

    public void resetFlagToFalse(){
        flag = false;
    }

    public void initTimer(int millis){
        timer = new Timer(millis, TimeUnit.MILLISECONDS);
        timer.start();
    }

    public boolean getTimerDone(){

        return timer != null && timer.done();
    }
}
