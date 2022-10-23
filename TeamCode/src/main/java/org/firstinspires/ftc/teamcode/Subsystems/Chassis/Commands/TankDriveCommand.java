package org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem.DRAWING_TARGET_RADIUS;
import static org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem.getValueWithDeadBand;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

import java.util.function.DoubleSupplier;

@Config
public class TankDriveCommand extends CommandBase {
    private final TankDriveSubsystem drive;
    private ToucheSubsystem toucheSubsystem;
    private LinearSystemSubsystem linearSystemSubsystem;
    public static boolean show_data_2_dash = false;
    public static double distance_to_high = 6;//Maybe y 7in

    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;
    private double deadBand = 0;
    private JustOnce trajectoryJustOnce;
    private boolean trajectoryEnabled = false;
    private boolean isAligned = false;

    private TrajectorySequence trajectory;

    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftY = leftY;
        this.rightX = rightX;

        trajectoryJustOnce = new JustOnce();
        addRequirements(drive);
    }

    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX, ToucheSubsystem toucheSubsystem) {
        this(drive, leftY, rightX);
        this.toucheSubsystem = toucheSubsystem;
    }

    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX, ToucheSubsystem toucheSubsystem, LinearSystemSubsystem linearSystemSubsystem) {
        this(drive, leftY, rightX, toucheSubsystem);
        //this.toucheSubsystem = toucheSubsystem;
        this.linearSystemSubsystem = linearSystemSubsystem;
    }

    public TankDriveCommand(TankDriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier rightX, double deadBand) {
        this(drive, leftY, rightX);
        this.deadBand = deadBand;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        switch (drive.getActualMode()) {
            case MANUAL_DRIVE:
                drive.setActualTrajectoryMode(DriveTrajectories.NON_AUTO);
                drive.drive(getValueWithDeadBand(leftY.getAsDouble(), 0.15), 0, rightX.getAsDouble());
                drive.setActualState(DriveStates.MANUAL_DRIVE);
                isAligned = false;
                break;

            case AUTONOMOUS_DRIVE:
                switch (drive.getActualTrajectory()) {
                    case SELF_POSITIONING_HUMAN_PLAYER:
                        trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(40)
                                .setReversed(false)
                                .splineTo(new Vector2d(-96, 119), drive.getPoseEstimate().getHeading())
                                //.back(70)
                                //.forward(70)
                                .build();
                        trajectoryEnabled = true;
                        //trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);
                        break;

                    case SELF_POSITIONING_CLIMBING:
                        trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setReversed(false)
                                .splineTo(new Vector2d(-22, 54), Math.toRadians(-45))
                                .turn(Math.toRadians(95)) //-24, 57
                                //.splineTo(new Vector2d(-10, 59), Math.toRadians(0))
                                //.back(70)
                                //.forward(70)
                                .build();

                        trajectoryEnabled = true;

                        //trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);

                        //if (!drive.isBusy()) {
                        //    drive.setActualTrajectoryMode(DriveTrajectories.ADVANCING_TO_SINK);
                        //}
                        break;

                    case ADVANCING_TO_SINK:
                        //if (toucheSubsystem.getActualState().equals(ToucheState.IS_POSITIONED_TO_CRASH))
                        if (toucheSubsystem.getActualState().equals(ToucheState.IS_ROBOT_POSITIONED)) {
                            drive.setActualState(DriveStates.IS_ALIGNED);
                            isAligned = true;
                            //drive.setMotorsPower(0, 0);
                            break;
                        }
                        if (!isAligned) {
                            drive.setActualState(DriveStates.IS_AUTO_ALIGNING);
                            drive.setMotorsPower(
                                    toucheSubsystem.isLeftDetecting() ? 0 : 0.2,
                                    toucheSubsystem.isRightDetecting() ? 0 : 0.2
                            );
                        }
                        trajectoryEnabled = false;
                        break;

                    case PREPARING_TO_CLIMB:
                        trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setReversed(false)
                                //.splineTo(new Vector2d(-10, 59), Math.toRadians(0))
                                //.back(70)
                                .forward(distance_to_high)
                                .build();

                        trajectoryEnabled = true;
                        break;

                    case NON_AUTO:
                        trajectoryEnabled = true;
                        break;
                }
                if (trajectory != null && trajectoryEnabled)
                    trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, trajectory, true)::schedule);
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

            case CANCEL_AUTONOMOUS_DRIVE:
                drive.setActualState(DriveStates.FINISHED_TRAJECTORIES);
                drive.breakTrajectoryFollowing();

                // if(!drive.isBusy())
                drive.setActualMode(DriveModes.MANUAL_DRIVE);
                break;

            case RELOCATE_DRIVE_DRIVERS_WALL:
                //drive.resetRobotPosInDriversWall();
                drive.resetRobotPosFelixSolution();
                //drive.setActualTrajectoryMode(DriveTrajectories.SELF_POSITIONING_CLIMBING);
                //drive.setActualMode(DriveModes.AUTONOMOUS_DRIVE);
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

            if (show_data_2_dash)
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            drive.getLocalizer().update();
            trajectoryJustOnce.resetFlagToFalse();
            //autonomousNeedAlign = false;
        }
        //trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);
    }
}

