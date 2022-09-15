package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes.*;
import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.BatteryChecker;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem_V2;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands.ClimberCommand;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class ClimberTeleOp_V2 extends ClockMode_V6 {
    ClimberSubsystem_V2 climberSubsystem;

    GamepadEx gamepadEx1;
    @Override
    public void initialize() {
        climberSubsystem = new ClimberSubsystem_V2(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        register(climberSubsystem);
        climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->climberSubsystem.setActualMode(ARM_UP));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->climberSubsystem.setActualMode(CRASHING_POSITION));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whenPressed(()->climberSubsystem.setActualMode(LIFT_ARM));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> climberSubsystem.setActualMode(CLIMB))
                .whenReleased(()-> climberSubsystem.setActualMode(OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> climberSubsystem.setActualMode(GO_DOWN))
                .whenReleased(()-> climberSubsystem.setActualMode(OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->climberSubsystem.setActualMode(INIT_POSITION));

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
