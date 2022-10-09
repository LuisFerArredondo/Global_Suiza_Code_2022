package org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;

@Config
public class ClimberLockCommand extends CommandBase {
    public static double open_Target= 0.5;
    public static double close_Target= 1;
    private ClimberLockSubsystem climberLockSubsystem;

    public ClimberLockCommand(ClimberLockSubsystem climberLockSubsystem) {
        this.climberLockSubsystem = climberLockSubsystem;

        addRequirements(climberLockSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (climberLockSubsystem.getActualMode()) {
            case OPEN:
                climberLockSubsystem.setLockPosition(open_Target);
                climberLockSubsystem.setActualState(LockStates.IS_OPENED);
                break;
            case CLOSE:
                climberLockSubsystem.setLockPosition(close_Target);
                climberLockSubsystem.setActualState(LockStates.IS_CLOSED);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
