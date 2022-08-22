package org.firstinspires.ftc.teamcode.Team15600Lib.Threads;

import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.ThreadState;

public class BlueThread extends Thread{
    private final boolean linearRunning;
    private final Runnable toRun;
    private ThreadState threadState;

    public BlueThread(boolean linearRunning, Runnable toRun){
        this.linearRunning = linearRunning;
        this.toRun = toRun;

        threadState = ThreadState.SLEEPING;
    }

    @Override
    public void run() {
        threadState = ThreadState.EXECUTING;
        do {
            if (Thread.currentThread().isInterrupted())
                break;

            toRun.run();
        } while (!linearRunning);

        threadState = ThreadState.FINISHED;
    }

    public ThreadState getThreadState(){
        return threadState;
    }
}
