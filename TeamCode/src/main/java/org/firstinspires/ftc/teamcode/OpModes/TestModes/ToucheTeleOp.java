package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands.ToucheCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V5;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;

@TeleOp
@Disabled
public class ToucheTeleOp extends ClockMode_V7 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;

    ToucheSubsystem toucheSubsystem;

    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);

        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        toucheSubsystem = new ToucheSubsystem(hardwareMap);

        register(tankDriveSubsystem, toucheSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx.getLeftY(), gamepadEx::getRightX, toucheSubsystem));
        toucheSubsystem.setDefaultCommand(new ToucheCommand(toucheSubsystem));

        new GamepadButton(gamepadEx, GamepadKeys.Button.A)
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.TOUCHE_MODE));

        new GamepadButton(gamepadEx, GamepadKeys.Button.B)
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.RETRACT));

        new GamepadButton(gamepadEx, GamepadKeys.Button.Y)
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTONOMOUS_DRIVE))
                .whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.ADVANCING_TO_SINK));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        enablePrintSensorsStates();
        enablePrintExtraStates();
        setChangeForEndGameTime(EXPOSITION_TIME);
        return CompetitionStages.TELEOP;
    }
}
