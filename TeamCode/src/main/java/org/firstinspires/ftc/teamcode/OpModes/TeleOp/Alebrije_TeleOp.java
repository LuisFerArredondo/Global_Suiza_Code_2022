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
import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands.PoweredClimberCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands.ClimberLockCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands.LinearSystemCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands.ToucheCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class Alebrije_TeleOp extends ClockMode_V7 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;

    ToucheSubsystem toucheSubsystem;
    ClimberSubsystem climberSubsystem;
    ClimberLockSubsystem climberLockSubsystem;
    LinearSystemSubsystem linearSystemSubsystem;

    IntakeSubsystem intakeSubsystem;
    GamepadEx gamepadEx1;
    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);

        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        toucheSubsystem = new ToucheSubsystem(hardwareMap);
        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        climberSubsystem = new ClimberSubsystem(hardwareMap);
        linearSystemSubsystem = new LinearSystemSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        register(tankDriveSubsystem, toucheSubsystem, climberSubsystem, climberLockSubsystem, linearSystemSubsystem, intakeSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx1.getLeftY(), gamepadEx1::getRightX, toucheSubsystem, linearSystemSubsystem));
        toucheSubsystem.setDefaultCommand(new ToucheCommand(toucheSubsystem));
        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));
        linearSystemSubsystem.setDefaultCommand(new LinearSystemCommand(linearSystemSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.RETRACT));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.X)
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.TOUCHE_MODE))
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.RELOCATE_DRIVE_DRIVERS_WALL));
                //.whenPressed(() -> tankDriveSubsystem.setActualTrajectoryMode(DriveTrajectories.SELF_POSITIONING_CLIMBING));

        //climber manual
        new GamepadButton(gamepadEx1, GamepadKeys.Button.Y)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.CLIMB))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        //Linear System Automatized
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()->linearSystemSubsystem.setActualMode(LinearSystemModes.AUTOMATIC_RETRACTION));
                //.whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        //new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
        //        .whileHeld(()->linearSystemSubsystem.setActualMode(LinearSystemModes.RETRACT_ARM))
        //        .whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        //ClimberLock
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(()->intakeSubsystem.setActualMode(IntakeMode.INTAKE))
                .whenReleased(()->intakeSubsystem.setActualMode(IntakeMode.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()->intakeSubsystem.setActualMode(IntakeMode.REJECT))
                .whenReleased(()->intakeSubsystem.setActualMode(IntakeMode.OFF));

    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        enableInitBeforeEndgame();
        enablePrintSensorsStates();
        //enablePrintExtraStates();

        setChangeForEndGameTime(EXPOSITION_TIME);
        return TELEOP;
    }

    @Override
    public void debugMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        toucheSubsystem = new ToucheSubsystem(hardwareMap);
        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        climberSubsystem = new ClimberSubsystem(hardwareMap);
        linearSystemSubsystem = new LinearSystemSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        register(tankDriveSubsystem, toucheSubsystem, climberSubsystem, climberLockSubsystem, linearSystemSubsystem, intakeSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx1.getLeftY(), gamepadEx1::getRightX, toucheSubsystem, linearSystemSubsystem));
        toucheSubsystem.setDefaultCommand(new ToucheCommand(toucheSubsystem));
        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));
        linearSystemSubsystem.setDefaultCommand(new LinearSystemCommand(linearSystemSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));


        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.RETRACT));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.X)
                .whenPressed(()-> toucheSubsystem.setActualMode(ToucheMode.TOUCHE_MODE));

        //climber manual
        new GamepadButton(gamepadEx1, GamepadKeys.Button.Y)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.CLIMB))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whileHeld(()->climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(()->climberSubsystem.setActualMode(ClimberModes.OFF));

        //Linear System Manual
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_UP)
                .whileHeld(()->linearSystemSubsystem.setActualMode(LinearSystemModes.LIFT_ARM))
                .whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(()->linearSystemSubsystem.setActualMode(LinearSystemModes.RETRACT_ARM))
                .whenReleased(()->linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        //ClimberLock
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(()->climberLockSubsystem.setActualMode(LockMode.CLOSE));

        //Manual Intake
        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(()->intakeSubsystem.setActualMode(IntakeMode.INTAKE))
                .whenReleased(()->intakeSubsystem.setActualMode(IntakeMode.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()->intakeSubsystem.setActualMode(IntakeMode.REJECT))
                .whenReleased(()->intakeSubsystem.setActualMode(IntakeMode.OFF));
    }
}
