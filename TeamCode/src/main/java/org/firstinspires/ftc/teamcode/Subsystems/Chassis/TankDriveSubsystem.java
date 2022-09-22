package org.firstinspires.ftc.teamcode.Subsystems.Chassis;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.Sensors.DistanceSensorUtil;

import java.util.List;

public class TankDriveSubsystem extends BrickSystem_V3 {

    private final SampleTankDrive drive;
    public static double DRAWING_TARGET_RADIUS = 2;
    private DriveStates actualState = DriveStates.DRIVE_STOPPED;
    private DriveModes actualMode = DriveModes.MANUAL_DRIVE;
    private DriveTrajectories actualTraj = DriveTrajectories.NON_AUTO;

    private final double robotRealTrackWidth = 14.5;//inch
    private final double distanceToSensor = 2;//inch

    //private DistanceSensor distanceSensor;
    private DistanceSensorUtil distanceSensorUtil;

    private PIDFController headingController = new PIDFController(SampleTankDrive.HEADING_PID);

    private Vector2d targetPosition = new Vector2d(0, 0);

    private Pose2d driveDirection = new Pose2d();

    private TelemetryPacket packet;
    private Canvas fieldOverlay;

    public TankDriveSubsystem(SampleTankDrive drive, HardwareMap hardwareMap) {
        this.drive = drive;

        headingController.setInputBounds(-Math.PI, Math.PI);
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();
        /*try {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "RLS");
        }catch (Exception e){
            RobotLog.dd("Distance Sensor State: ", "not Found, " + e);
        }*/
        distanceSensorUtil = new DistanceSensorUtil(hardwareMap, "RLS", ColorFormatter.ORANGE, "Distance S");

        setSubsystemSensors(distanceSensorUtil);
        setSubsystemTelemetryColor(ColorFormatter.ORANGE);
        setSubsystemState(actualState.toString());
        setName("Drive");
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

    public void autoAlignDrive(double leftY, double leftX, double rightX, Vector2d targetPosition){
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


        if(Math.abs(headingController.getLastError()) < 0.2)
            setActualState(DriveStates.IS_ALIGNED);
        else setActualState(DriveStates.IS_AUTO_ALIGNING);
    }

    public void autoAlignDrive(double leftY, double leftX, double rightX){
        autoAlignDrive(leftY, leftX, rightX, this.targetPosition);
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

    public Vector2d getTargetPosition(){return targetPosition;}

    public void resetRobotPosInDriversWall(){
        setActualState(DriveStates.IS_RELOCATING);
        double distance = distanceSensorUtil.getDistance(INCH);

        double hypotenuse = distance + (robotRealTrackWidth / 2) + distanceToSensor;

        double RTheta = Math.IEEEremainder(Math.toDegrees(getPoseEstimate().getHeading()), 360);
        double RAlpha = RTheta + 90;
        //por alguna razon me lo da en grados la puta

        double getX = (hypotenuse * (Math.cos(Math.toRadians(RAlpha))));
        double getY = (getX * (Math.tan(Math.toRadians(RTheta))));

        double FinalY = 137.795 - Math.abs(getY);
        double FinalX = 118.11 - getX;
        //            137.8 -
        setPoseEstimate(new Pose2d(-FinalX, FinalY, getPoseEstimate().getHeading()));
        //setActualState(DriveStates.IS_RELOCATED);
    }

    public void resetRobotPosInOppositeWall(){
        setActualState(DriveStates.IS_RELOCATING);
        double distance = distanceSensorUtil.getDistance(INCH);

        double hypotenuse = distance + (robotRealTrackWidth / 2) + distanceToSensor;

        double RTheta = Math.IEEEremainder(Math.toDegrees(getPoseEstimate().getHeading()), 360);
        double RAlpha = RTheta + 90;
        //por alguna razon me lo da en grados la puta

        double getY = (hypotenuse * (Math.sin(Math.toRadians(RAlpha))));
        double getX = (getY / (Math.tan(Math.toRadians(RTheta))));

        double FinalY = 137.795 - Math.abs(getY);
        double FinalX = 118.11 - Math.abs(getX);
        //            137.8 -

        setPoseEstimate(new Pose2d(-FinalX, FinalY, getPoseEstimate().getHeading() + Math.toRadians(180)));
        //setActualState(DriveStates.IS_RELOCATED);
    }


    //Para añadir valores a la dashboard    53 inch la diagonal del compressor
    //public static void addPacket(String key, Object value) {
    //    packet.put(key, value.toString());
    //}

    public DriveTrajectories getActualTrajectory() {
        return actualTraj;
    }

    public void setActualTrajectoryMode(DriveTrajectories trajectory) {
        this.actualTraj = trajectory;
    }

    public double getBatteryVoltage(){
        return drive.getBatteryVoltage();
    }

    public static double getValueWithDeadBand(double value, double deadBand){
        return Math.abs(value) > deadBand ? value : 0;
    }

    public void setMotorsPower(double leftMotor, double rightMotor){
        drive.setMotorPowers(leftMotor, rightMotor);
    }
    @Override
    public void periodic() {
        //double distance = getDetectedDistance();
        //double hypotenuse = distance + (robotRealTrackWidth / 2);

        setSubsystemState(getActualState().toString());
        distanceSensorUtil.changeState();
                //+ "\nHeading Controller Errors: " + headingController.getLastError()
                //+ "\nDistance Sensor detected distance: " + distance
                //+ "\nHypotenous measurement " + hypotenuse);
    }
}
