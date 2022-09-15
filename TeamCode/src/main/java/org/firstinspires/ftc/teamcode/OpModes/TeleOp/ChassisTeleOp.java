package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class ChassisTeleOp extends ClockMode_V6 {
    private SampleTankDrive sampleTankDrive;
    private TankDriveSubsystem tankDriveSubsystem;
    private GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        register(tankDriveSubsystem);
        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> -gamepadEx1.getLeftY(), gamepadEx1::getRightX));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .toggleWhenActive(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTO_ALIGN_DRIVE),
                        () -> tankDriveSubsystem.setActualMode(DriveModes.MANUAL_DRIVE));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(()-> tankDriveSubsystem.setActualMode(DriveModes.CANCEL_DRIVE))
                .whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.NON_AUTO));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.Y)
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.RELOCATE_DRIVE_DRIVERS_WALL));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        setChangeForEndGameTime(EXPOSITION_TIME);
        return TELEOP;
    }

}
