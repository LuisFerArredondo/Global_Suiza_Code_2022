package org.firstinspires.ftc.teamcode.Team15600Lib.Threads;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;

public class DeadLineStateCommand extends CommandBase {
    private final String deadLineState;
    private final BrickSystem_V3 subsystem;
    private Runnable modeChange;

    public DeadLineStateCommand(BrickSystem_V3 subsystem, String deadLineState){
        this.subsystem = subsystem;
        this.deadLineState = deadLineState;
        modeChange = ()->{};
    }

    public DeadLineStateCommand(BrickSystem_V3 subsystem, String deadLineState, Runnable toRun){
        this(subsystem, deadLineState);
        this.modeChange = toRun;
    }

    @Override
    public void initialize() {
        modeChange.run();
    }

    @Override
    public boolean isFinished() {
        return subsystem.getSubsystemState().equals(deadLineState);
    }
}
