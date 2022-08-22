package org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.DriveStates;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankDriveSubsystem drive;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;


    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        switch (drive.getActualMode()){
            case MANUAL_DRIVE:
                drive.drive(leftY.getAsDouble(), 0, rightX.getAsDouble());
                drive.setActualState(DriveStates.MANUAL_DRIVE);
                break;

            case AUTONOMOUS_DRIVE:
               TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                       .splineTo(new Vector2d(0,-46),Math.toRadians(-90))
                        .build();

                new TrajectoryFollowerCommand(drive, traj, true).schedule();
                break;

            case AUTO_ALIGN_DRIVE:
                drive.autoAlignDrive(leftY.getAsDouble(),0,rightX.getAsDouble());
                drive.setActualState(DriveStates.IS_AUTO_ALIGNING);
                break;
        }
    }
}

