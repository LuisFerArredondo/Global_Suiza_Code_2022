package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

public class JustOnce {
    private boolean flag = false;

    public JustOnce(){}

    public void JustOneTime(boolean trigger, Runnable toRun){
        if(trigger && !flag){
            toRun.run();
            flag = true;
        }
        if(!trigger) flag = false;
    }
}
