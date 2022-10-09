package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands.ClimberLockCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands.LinearSystemCommand;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled

public class ClimberLockTest extends ClockMode_V7 {
    ClimberLockSubsystem climberLockSubsystem;

    GamepadEx gamepadEx;

    @Override
    public void initialize() {
        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        gamepadEx = new GamepadEx(gamepad1);

        register(climberLockSubsystem);
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));

        new GamepadButton(gamepadEx, GamepadKeys.Button.A)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(gamepadEx, GamepadKeys.Button.B)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        enablePrintExtraStates();
        enablePrintSensorsStates();
        return CompetitionStages.TELEOP;
    }
}
