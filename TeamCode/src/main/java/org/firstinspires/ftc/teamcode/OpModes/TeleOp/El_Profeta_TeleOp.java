package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands.LinearSystemCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands.ToucheCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

@TeleOp
public class El_Profeta_TeleOp extends ClockMode_V7 {
    private SampleTankDrive sampleTankDrive;
    private TankDriveSubsystem tankDriveSubsystem;
    private ClimberSubsystem climberSubsystem;
    private ClimberLockSubsystem climberLockSubsystem;
    private LinearSystemSubsystem linearSystemSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ToucheSubsystem toucheSubsystem;
    private RobotState robotState;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private JustOnce mexicanFlag;
    private Command mexicanFlagCommand;
    private Command autoHookDepose;

    @Override
    public void initialize() {
        //gamepad2.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);

        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        climberSubsystem = new ClimberSubsystem(hardwareMap);
        climberLockSubsystem = new ClimberLockSubsystem(hardwareMap);
        linearSystemSubsystem = new LinearSystemSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        toucheSubsystem = new ToucheSubsystem(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        mexicanFlag = new JustOnce();

        robotState = new RobotState(tankDriveSubsystem, climberSubsystem, climberLockSubsystem, intakeSubsystem, linearSystemSubsystem, toucheSubsystem, gamepadEx1);

        gamepad1.setLedColor(128, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
        robotState.ChangeLedColor2MexicanFlag(gamepad2);//---------------------------------------

        mexicanFlagCommand  = robotState.mexicanFlagCommand(gamepad2);
        autoHookDepose = robotState.automaticHookDepose();

        register(tankDriveSubsystem, climberSubsystem, climberLockSubsystem, linearSystemSubsystem, intakeSubsystem, toucheSubsystem);

        robotState.enableSubsystemsLoops();

        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(robotState.intakeCarbons());

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(robotState.rejectCarbons());

        new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_UP)
                .whileHeld(robotState.dpadUpCommand());
                //.whileHeld(changeGamepadConfig ? robotState.ManualLift() : new InstantCommand(() -> robotState.compressorTrajectory().schedule()));

        new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(robotState.rightBumperCommand());

        new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()->robotState.setToucheMode(ToucheMode.RETRACT));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whileHeld(()->robotState.setToucheMode(ToucheMode.RETRACT));

        //Automatic Hook Depose
        //new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_RIGHT)
        //        .whenPressed(autoHookDepose);
        new GamepadButton(gamepadEx1, GamepadKeys.Button.X)
                .whenPressed(autoHookDepose);

        //Automatic Climb
        new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(robotState.dpadDownCommand());

        //ClimberLock
        new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robotState.setClimberLockMode(LockMode.OPEN));

        new GamepadButton(gamepadEx2, GamepadKeys.Button.X)
                .whenPressed(()->robotState.setClimberLockMode(LockMode.CLOSE));
                //.whenPressed(robotState.closeLock());

        //Cancel Autonomous Routines
        new GamepadButton(gamepadEx2, GamepadKeys.Button.B)
                .whenPressed(() ->{
                    robotState.cancelAutonomousRoutines();
                    autoHookDepose.cancel();
                });
        //Cancel Autonomous Routines
        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(() ->{
                    robotState.cancelAutonomousRoutines();
                    autoHookDepose.cancel();
                });

        //climber manual
        new GamepadButton(gamepadEx2, GamepadKeys.Button.Y)
                .whileHeld(robotState.ManualClimb());

        new GamepadButton(gamepadEx2, GamepadKeys.Button.A)
                .whileHeld(robotState.ManualDown());

        schedule(new RunCommand(()->{
            if(!mexicanFlagCommand.isScheduled())
                mexicanFlag.JustOneTime(true,()->{
                    mexicanFlagCommand.schedule();
                });

            if(mexicanFlagCommand.isFinished())
                mexicanFlag.resetFlagToFalse();

                setExtraStates("buebos","is auto hook scheduled: " + autoHookDepose.isScheduled());
        }));
        //schedule(new InstantCommand(()->robotState.mexicanFlagComamnd(gamepad2).schedule()));

    }
/*
    @Override
    public Runnable whenTeleOp() {
        return ()->{
            while (Thread.currentThread().isInterrupted());
        };
    }
    /*@Override
    public Runnable whenTeleOp() {
        return () -> {
            while (!Thread.currentThread().isInterrupted()) {

                right_bumper.whileHeld(gamepad2.right_bumper, () -> changeGamepadConfig = true)
                        .whenReleased(gamepad2.right_bumper, () -> changeGamepadConfig = false);

                dpad_up.whileHeld(gamepad2.dpad_up && changeGamepadConfig, robotState.ManualLift())
                        .whenPressed(gamepad2.dpad_up && !changeGamepadConfig, robotState.compressorTrajectory());

                dpad_down.whileHeld(gamepad2.dpad_down && changeGamepadConfig, robotState.ManualRetract())
                        .whenPressed(gamepad2.dpad_down && !changeGamepadConfig, robotState.automaticClimb());

            }
        };
    }*/

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        setChangeForEndGameTime(105);
        enablePrintSensorsStates();
        enableInitBeforeEndgame();
        enablePrintExtraStates();
        disableDebuggingMode();
        return CompetitionStages.TELEOP;
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
                .whenPressed(() -> toucheSubsystem.setActualMode(ToucheMode.TOUCHE_MODE));

        //climber manual
        new GamepadButton(gamepadEx1, GamepadKeys.Button.Y)
                .whileHeld(() -> climberSubsystem.setActualMode(ClimberModes.CLIMB))
                .whenReleased(() -> climberSubsystem.setActualMode(ClimberModes.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .whileHeld(() -> climberSubsystem.setActualMode(ClimberModes.GO_DOWN))
                .whenReleased(() -> climberSubsystem.setActualMode(ClimberModes.OFF));

        //Linear System Manual
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_UP)
                .whileHeld(() -> linearSystemSubsystem.setActualMode(LinearSystemModes.LIFT_ARM))
                .whenReleased(() -> linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(() -> linearSystemSubsystem.setActualMode(LinearSystemModes.RETRACT_ARM))
                .whenReleased(() -> linearSystemSubsystem.setActualMode(LinearSystemModes.OFF));

        //ClimberLock
        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> climberLockSubsystem.setActualMode(LockMode.OPEN));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> climberLockSubsystem.setActualMode(LockMode.CLOSE));

        //Manual Intake
        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> intakeSubsystem.setActualMode(IntakeMode.INTAKE))
                .whenReleased(() -> intakeSubsystem.setActualMode(IntakeMode.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> intakeSubsystem.setActualMode(IntakeMode.REJECT))
                .whenReleased(() -> intakeSubsystem.setActualMode(IntakeMode.OFF));
    }

}
