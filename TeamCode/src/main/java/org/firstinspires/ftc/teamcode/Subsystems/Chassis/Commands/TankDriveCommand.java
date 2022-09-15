package org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem.DRAWING_TARGET_RADIUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankDriveSubsystem drive;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;
    private JustOnce trajectoryJustOnce;

    private TrajectorySequence traj;

    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = leftY;
        this.rightX = rightX;

        trajectoryJustOnce = new JustOnce();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (drive.getActualMode()) {
            case MANUAL_DRIVE:
                drive.drive(leftY.getAsDouble(), 0, rightX.getAsDouble());
                drive.setActualState(DriveStates.MANUAL_DRIVE);
                break;

            case AUTONOMOUS_DRIVE:
                switch (drive.getActualTrajectory()){
                    case SELF_POSITIONING_HUMAN_PLAYER:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(40)
                                .setReversed(false)
                                .splineTo(new Vector2d(-96,119), drive.getPoseEstimate().getHeading())
                                //.back(70)
                                //.forward(70)
                                .build();

                        trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);
                        break;

                    case SELF_POSITIONING_CLIMBING:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setReversed(false)
                                .splineTo(new Vector2d(-1,53), Math.toRadians(0))
                                //.back(70)
                                //.forward(70)
                                .build();

                        trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);

                        if(!drive.isBusy()){
                            drive.setActualTrajectoryMode(DriveTrajectories.ADVANCING_TO_SINK);
                        }
                        break;

                    case ADVANCING_TO_SINK:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setReversed(false)
                                //.splineTo(new Vector2d(-1,53), Math.toRadians(0))
                                //.back(70)
                                .forward(70)
                                .build();

                        trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);
                        break;

                    case PREPARING_TO_CLIMB:
                        break;
                    case NON_AUTO:
                        break;
                }
                break;

            case AUTO_ALIGN_DRIVE:
                // Draw the target on the field
                fieldOverlay.setStroke(drive.getSubsystemState().equals(DriveStates.IS_ALIGNED.toString()) ? "#00FF1D" : "#dd2c00");
                fieldOverlay.strokeCircle(drive.getTargetPosition().getX(), drive.getTargetPosition().getY(), DRAWING_TARGET_RADIUS);

                // Draw lines to target
                fieldOverlay.setStroke(drive.getSubsystemState().equals(DriveStates.IS_ALIGNED.toString()) ? "#00FF1D" : "#b89eff");
                fieldOverlay.strokeLine(drive.getTargetPosition().getX(), drive.getTargetPosition().getY(), drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());

                fieldOverlay.setStroke(drive.getSubsystemState().equals(DriveStates.IS_ALIGNED.toString()) ? "#00FF1D" : "#ffce7a");
                fieldOverlay.strokeLine(drive.getTargetPosition().getX(), drive.getTargetPosition().getY(), drive.getTargetPosition().getX(), drive.getPoseEstimate().getY());
                fieldOverlay.strokeLine(drive.getTargetPosition().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());

                drive.autoAlignDrive(leftY.getAsDouble(), 0, rightX.getAsDouble());
                break;

            case CANCEL_DRIVE:
                drive.breakTrajectoryFollowing();
                drive.setActualMode(DriveModes.MANUAL_DRIVE);
                break;

            case RELOCATE_DRIVE_DRIVERS_WALL:
                drive.resetRobotPosInDriversWall();
                drive.setActualMode(DriveModes.MANUAL_DRIVE);
                break;

            case RELOCATE_DRIVE_OPPOSITE_WALL:
                drive.resetRobotPosInOppositeWall();
                drive.setActualMode(DriveModes.MANUAL_DRIVE);
                break;
        }
        // Draw bot on canvas
        // x = 6m Total, y = 7m total
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, drive.getPoseEstimate());

        if (!drive.getActualMode().equals(DriveModes.AUTONOMOUS_DRIVE)) {
            packet.put("Drive States", drive.getSubsystemState());
            packet.put("Drive x", drive.getPoseEstimate().getX());
            packet.put("Drive y", drive.getPoseEstimate().getY());
            packet.put("Drive heading", Math.IEEEremainder(Math.toDegrees(drive.getPoseEstimate().getHeading()), 360));
            packet.put("Battery Voltage", drive.getBatteryVoltage());

           // FtcDashboard.getInstance().sendTelemetryPacket(packet);
            drive.getLocalizer().update();
            trajectoryJustOnce.resetFlagToFalse();
            //autonomousNeedAlign = false;
        }
    }
}

