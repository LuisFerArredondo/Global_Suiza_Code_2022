package org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.DriveStates;

public class TrajectoryFollowerCommand extends CommandBase {

    private final TankDriveSubsystem drive;
    private final TrajectorySequence trajectory;
    private final boolean async;

    public TrajectoryFollowerCommand(TankDriveSubsystem drive, TrajectorySequence trajectory, boolean Async) {
        this.drive = drive;
        this.trajectory = trajectory;
        async = Async;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (async)
            drive.followTrajectorySequenceAsync(trajectory);
        else
            drive.followTrajectorySequence(trajectory);

        drive.setActualState(DriveStates.INITIALIZING_TRAJECTORY);
    }

    @Override
    public void execute() {
        drive.setActualState(DriveStates.FOLLOWING_TRAJECTORY);
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setActualMode(DriveModes.MANUAL_DRIVE);
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}