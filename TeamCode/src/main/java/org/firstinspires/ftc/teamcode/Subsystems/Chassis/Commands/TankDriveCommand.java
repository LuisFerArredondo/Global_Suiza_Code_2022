package org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem.DRAWING_TARGET_RADIUS;
import static org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem.getValueWithDeadBand;

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
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {

    private final TankDriveSubsystem drive;
    private ToucheSubsystem toucheSubsystem;
    private LinearSystemSubsystem linearSystemSubsystem;

    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;
    private double deadBand = 0;
    private JustOnce trajectoryJustOnce;

    private TrajectorySequence traj;

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
                break;

            case AUTONOMOUS_DRIVE:
                switch (drive.getActualTrajectory()) {
                    case SELF_POSITIONING_HUMAN_PLAYER:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(40)
                                .setReversed(false)
                                .splineTo(new Vector2d(-96, 119), drive.getPoseEstimate().getHeading())
                                //.back(70)
                                //.forward(70)
                                .build();

                        trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);
                        break;

                    case SELF_POSITIONING_CLIMBING:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .setReversed(false)
                                .splineTo(new Vector2d(-1, 53), Math.toRadians(0))
                                //.back(70)
                                //.forward(70)
                                .build();

                        trajectoryJustOnce.JustOneTime(true, new TrajectoryFollowerCommand(drive, traj, true)::schedule);

                        if (!drive.isBusy()) {
                            drive.setActualTrajectoryMode(DriveTrajectories.ADVANCING_TO_SINK);
                        }
                        break;

                    case ADVANCING_TO_SINK:
                        //if (toucheSubsystem.getActualState().equals(ToucheState.IS_POSITIONED_TO_CRASH))
                        if (toucheSubsystem.getActualState().equals(ToucheState.IS_ROBOT_POSITIONED)){
                            drive.setActualTrajectoryMode(DriveTrajectories.PREPARING_TO_CLIMB);
                            break;
                        }
                            drive.setMotorsPower(
                                    toucheSubsystem.isLeftDetecting() ? 0 : -0.2
                                    , toucheSubsystem.isRightDetecting()? 0 : -0.2
                            );
                        break;

                    case PREPARING_TO_CLIMB:
                        linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_ARM_UP);
                        drive.setActualMode(DriveModes.MANUAL_DRIVE);
                        //if (!linearSystemSubsystem.isMotorBusy())
                        //drive.setActualTrajectoryMode(DriveTrajectories.HOOK_DEPOSED_AND_CLIMBING);
                        break;

                    case HOOK_DEPOSED_AND_CLIMBING:
                        linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_RETRACTION);

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

            //FtcDashboard.getInstance().sendTelemetryPacket(packet);
            drive.getLocalizer().update();
            trajectoryJustOnce.resetFlagToFalse();
            //autonomousNeedAlign = false;
        }
    }
}

