package org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RoadRunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

import java.util.List;

public class TankDriveSubsystem extends BrickSystem_V2 {

    private final SampleTankDrive drive;
    public static double DRAWING_TARGET_RADIUS = 2;
    private DriveStates actualState = DriveStates.DRIVE_STOPPED;
    private DriveModes actualMode = DriveModes.MANUAL_DRIVE;

    private PIDFController headingController = new PIDFController(SampleTankDrive.HEADING_PID);

    private Vector2d targetPosition = new Vector2d(0, 0);

    private Pose2d driveDirection = new Pose2d();
    private TelemetryPacket packet;
    private Canvas fieldOverlay;

    public TankDriveSubsystem(SampleTankDrive drive) {
        this.drive = drive;

        headingController.setInputBounds(-Math.PI, Math.PI);
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();

        setSubsystemTelemetryColor(ColorFormatter.ORANGE);
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        // Draw the target on the field
        drive.setWeightedDrivePower(
                new Pose2d(
                        -leftY,
                        0 ,
                        -rightX
                )
        );
    }

    public void autoAlignDrive(double leftY, double leftX, double rightX){
// Create a vector from the gamepad x/y inputs which is the field relative movement
        // Then, rotate that vector by the inverse of that heading for field centric control
        Vector2d fieldFrameInput = new Vector2d(
                -leftY,
                -leftX
        );
        Vector2d robotFrameInput = fieldFrameInput.rotated(-getPoseEstimate().getHeading());

        // Difference between the target vector and the bot's position
        Vector2d difference = targetPosition.minus(getPoseEstimate().vec());
        // Obtain the target angle for feedback and derivative for feedforward
        double theta = difference.angle();

        // Not technically omega because its power. This is the derivative of atan2
        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

        // Set the target heading for the heading controller to our desired angle
        headingController.setTargetPosition(theta);

        // Set desired angular velocity to the heading controller output + angular
        // velocity feedforward
        double headingInput = (headingController.update(getPoseEstimate().getHeading())
                * DriveConstants.kV + thetaFF)
                * DriveConstants.TRACK_WIDTH;

        // Combine the field centric x/y velocity with our derived angular velocity
        driveDirection = new Pose2d(
                robotFrameInput.getX(),0,
                headingInput
        );


        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(getPoseEstimate().getHeading());

        // Update he localizer
        drive.getLocalizer().update();
    }


    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectorySequence(TrajectorySequence trajectory) {
        drive.followTrajectorySequence(trajectory);
    }
    public void followTrajectorySequenceAsync(TrajectorySequence trajectory) {
        drive.followTrajectorySequenceAsync(trajectory);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public DriveModes getActualMode() {
        return actualMode;
    }

    public void setActualMode(DriveModes actualMode) {
        this.actualMode = actualMode;
    }

    public DriveStates getActualState() {
        return actualState;
    }

    public void setActualState(DriveStates actualState) {
        this.actualState = actualState;
    }

    public void breakTrajectoryFollowing(){
        drive.breakFollowing();
    }

    public void drawAutoAlign(){
        // Draw the target on the field
        fieldOverlay.setStroke(!getActualState().equals(DriveStates.IS_ALIGNED)?"#FF2222":"#00D318");
        fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

        // Draw lines to target
        fieldOverlay.setStroke(!getActualState().equals(DriveStates.IS_ALIGNED)?"#b89eff" : "#00D318");
        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), getPoseEstimate().getX(), getPoseEstimate().getY());
        fieldOverlay.setStroke(!getActualState().equals(DriveStates.IS_ALIGNED)?"#ffce7a" : "#00D318");
        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), getPoseEstimate().getY());
        fieldOverlay.strokeLine(targetPosition.getX(), getPoseEstimate().getY(), getPoseEstimate().getX(), getPoseEstimate().getY());
    }

    public void drawBot(){
        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate());
    }

    @Override
    public String setSubsystemState() {
        return getActualState().toString();
    }


}
