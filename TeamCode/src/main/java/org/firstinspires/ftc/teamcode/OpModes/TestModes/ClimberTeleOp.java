package org.firstinspires.ftc.teamcode.OpModes.TestModes;

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
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands.LinearSystemCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V5;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled

public class ClimberTeleOp extends ClockMode_V7 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;

    ClimberSubsystem climberSubsystem;
    ClimberLockSubsystem climberLockSubsystem;
    LinearSystemSubsystem linearSystemSubsystem;

    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);

        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        climberSubsystem = new ClimberSubsystem(hardwareMap);
        linearSystemSubsystem = new LinearSystemSubsystem(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);

        register(tankDriveSubsystem, climberSubsystem, climberLockSubsystem, linearSystemSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx1.getLeftY(), gamepadEx1::getRightX));
        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));
        linearSystemSubsystem.setDefaultCommand(new LinearSystemCommand(linearSystemSubsystem));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y)
                .whenPressed(()->climberSubsystem.setActualMode(ClimberModes.AUTOMATIC_CLIMB));
                //.whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

/*
        //Linear System by power
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_ARM_UP));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_RETRACTION));
*/

        //Linear System Automatized
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_ARM_UP));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_RETRACTION));

        //ClimberLock
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        //enableDebuggingMode();
        enablePrintSensorsStates();
        enablePrintExtraStates();
        setChangeForEndGameTime(EXPOSITION_TIME);
        return CompetitionStages.TELEOP;
    }

    @Override
    public void debugMode() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);

        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        climberSubsystem = new ClimberSubsystem(hardwareMap);
        linearSystemSubsystem = new LinearSystemSubsystem(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);

        register(tankDriveSubsystem, climberSubsystem, climberLockSubsystem, linearSystemSubsystem);

        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx1.getLeftY(), gamepadEx1::getRightX));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));
        linearSystemSubsystem.setDefaultCommand(new LinearSystemCommand(linearSystemSubsystem));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.Y)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.CLIMB))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

/*
        //Linear System by power
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_ARM_UP));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_RETRACTION));
*/

        //Linear System Automatized
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whileHeld(()->linearSystemSubsystem.setActualMode(LinearSystemModes.LIFT_ARM))
                .whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(()->linearSystemSubsystem.setActualMode(LinearSystemModes.RETRACT_ARM))
                .whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        //ClimberLock
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.X)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));
    }
}
