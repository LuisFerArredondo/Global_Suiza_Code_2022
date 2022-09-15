package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands.PoweredClimberCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands.ClimberLockCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V5;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
public class ClimberTeleOp extends ClockMode_V6 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;

    ClimberSubsystem climberSubsystem;
    ClimberLockSubsystem climberLockSubsystem;

    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        climberSubsystem = new ClimberSubsystem(hardwareMap);

        register(tankDriveSubsystem, climberSubsystem, climberLockSubsystem);
        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx1.getLeftY(), gamepadEx1::getRightX));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.CLIMB))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.LIFT_ARM))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.RETRACT_ARM))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));
                //.whenPressed(()->climberSubsystem.setActualMode(ClimberModes.OFF));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        setChangeForEndGameTime(EXPOSITION_TIME);
        return CompetitionStages.TELEOP;
    }
}
