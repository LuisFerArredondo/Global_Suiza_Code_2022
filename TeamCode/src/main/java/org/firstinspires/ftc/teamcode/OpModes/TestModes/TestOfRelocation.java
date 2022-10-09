package org.firstinspires.ftc.teamcode.OpModes.TestModes;

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
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class TestOfRelocation extends ClockMode_V6 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;

    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        register(tankDriveSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> -gamepadEx1.getLeftY(), gamepadEx1::getRightX));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.X)
                .whenPressed(()->tankDriveSubsystem.setActualMode(DriveModes.RELOCATE_DRIVE_DRIVERS_WALL));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(()->tankDriveSubsystem.setActualMode(DriveModes.RELOCATE_DRIVE_OPPOSITE_WALL));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTONOMOUS_DRIVE))
                .whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.SELF_POSITIONING_HUMAN_PLAYER));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTONOMOUS_DRIVE))
                .whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.SELF_POSITIONING_CLIMBING));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whenPressed(()-> tankDriveSubsystem.setActualMode(DriveModes.CANCEL_AUTONOMOUS_DRIVE))
                .whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.NON_AUTO));

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
