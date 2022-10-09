package org.firstinspires.ftc.teamcode.Team15600Lib.Util;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.BlueThread;

import java.util.concurrent.TimeUnit;

public class JustOnce {
    private boolean flag = false;
    private boolean flag2 = false;
    private Thread thread1;
    private Thread thread2;


    public JustOnce(){}


    public void JustOneTime(boolean trigger, Runnable toRun){
        if(trigger && !flag){
            toRun.run();
            flag = true;
        }
        if(!trigger) flag = false;
    }

    public void JustOneTimeHeld(boolean trigger, Runnable toRun1, Runnable toRun2){
        if(trigger && !flag){
            toRun1.run();
            flag = true;
            flag2 = false;
        }else if(!trigger && !flag2){

            toRun2.run();
            flag2 = true;
        }

        if(!trigger) flag = false;
            //flag2 = true;
    }
    public void loopRunning(Runnable toRun){
        toRun.run();
    }

    public void resetFlagToFalse(){
        flag = false;
    }


}
