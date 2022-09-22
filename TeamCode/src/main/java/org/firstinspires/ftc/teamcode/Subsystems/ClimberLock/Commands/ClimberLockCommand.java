package org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;

public class ClimberLockCommand extends CommandBase {
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
                climberLockSubsystem.setLockPosition(0);
                climberLockSubsystem.setActualState(LockStates.IS_OPENED);
                break;
            case CLOSE:
                climberLockSubsystem.setLockPosition(1);
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
