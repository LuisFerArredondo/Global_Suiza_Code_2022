package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveTrajectories;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands.PoweredClimberCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Commands.ClimberLockCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands.LinearSystemCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands.ToucheCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheMode;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.DeadLineStateCommand;

import java.util.concurrent.TimeUnit;

public class RobotState {
    private final TankDriveSubsystem tankDriveSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final ClimberLockSubsystem climberLockSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LinearSystemSubsystem linearSystemSubsystem;
    private final ToucheSubsystem toucheSubsystem;
    private final GamepadEx gamepadEx;
    private boolean changeGamepadConfig = false;

    private Timer climberTimer;
    private Timer timer = new Timer(5700, TimeUnit.MILLISECONDS);

    private double driveMultiplier = 1;


    public RobotState(TankDriveSubsystem tankDriveSubsystem, ClimberSubsystem climberSubsystem,
                      ClimberLockSubsystem climberLockSubsystem, IntakeSubsystem intakeSubsystem,
                      LinearSystemSubsystem linearSystemSubsystem, ToucheSubsystem toucheSubsystem, GamepadEx gamepadEx) {

        this.tankDriveSubsystem = tankDriveSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.climberLockSubsystem = climberLockSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.linearSystemSubsystem = linearSystemSubsystem;
        this.toucheSubsystem = toucheSubsystem;
        this.gamepadEx = gamepadEx;

        climberTimer = new Timer(500, TimeUnit.MILLISECONDS);
    }

    public void ChangeLedColor2MexicanFlag(Gamepad gamepad) {
        int time = 1000;
        Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                //Verde
                //.addStep(0,182,0,time)
                .addStep(0, 255, 0, time)
                //.addStep(176, 255, 180, time)
                //Blanco
                .addStep(255, 255, 255, time)
                //Rojo
                //.addStep(255, 176,176, time)
                .addStep(255, 0, 0, time + 1000)
                //.addStep(193, 0,0, time)
                .build();

        gamepad.runLedEffect(ledEffect);

    }

    public void enableSubsystemsLoops() {
        toucheSubsystem.setDefaultCommand(new ToucheCommand(toucheSubsystem));
        climberSubsystem.setDefaultCommand(new PoweredClimberCommand(climberSubsystem));
        climberLockSubsystem.setDefaultCommand(new ClimberLockCommand(climberLockSubsystem));
        linearSystemSubsystem.setDefaultCommand(new LinearSystemCommand(linearSystemSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> gamepadEx.getLeftY() * driveMultiplier, gamepadEx::getRightX, toucheSubsystem));
    }

    public void disableSubsystemsLoops() {
        tankDriveSubsystem.getDefaultCommand().cancel();
        toucheSubsystem.getDefaultCommand().cancel();
        climberSubsystem.getDefaultCommand().cancel();
        climberLockSubsystem.getDefaultCommand().cancel();
        linearSystemSubsystem.getDefaultCommand().cancel();
        intakeSubsystem.getDefaultCommand().cancel();
    }

    public void invertDrive() {
        driveMultiplier *= -1;
    }

    public void setDriveMode(DriveModes driveMode) {
        tankDriveSubsystem.setActualMode(driveMode);
    }

    public void setDriveTrajectoryMode(DriveTrajectories trajectories) {
        tankDriveSubsystem.setActualTrajectoryMode(trajectories);
    }

    public void setClimberMode(ClimberModes climberModes) {
        climberSubsystem.setActualMode(climberModes);
    }

    public void setClimberLockMode(LockMode lockMode) {
        climberLockSubsystem.setActualMode(lockMode);
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        intakeSubsystem.setActualMode(intakeMode);
    }

    public void setLinearMode(LinearSystemModes linearMode) {
        linearSystemSubsystem.setActualMode(linearMode);
    }

    public void setToucheMode(ToucheMode toucheMode) {
        toucheSubsystem.setActualMode(toucheMode);
    }

    public CommandBase mexicanFlagCommand(Gamepad gamepad) {
        return new CommandBase() {

            @Override
            public void initialize() {
                int time = 100;

                Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                        //Verde
                        //.addStep(0,182,0,time)
                        .addStep(0, 0.1, 0, time)
                        .addStep(0, 0.2, 0, time)
                        .addStep(0, 0.3, 0, time)
                        .addStep(0, 0.4, 0, time)
                        .addStep(0, 0.5, 0, time)
                        .addStep(0, 0.6, 0, time)
                        .addStep(0, 0.7, 0, time)
                        .addStep(0, 0.8, 0, time)
                        .addStep(0, 0.9, 0, time)
                        .addStep(0, 1, 0, time)
                        .addStep(0, 0.9, 0, time)
                        .addStep(0, 0.8, 0, time)
                        .addStep(0, 0.7, 0, time)
                        .addStep(0, 0.6, 0, time)
                        .addStep(0, 0.5, 0, time)
                        .addStep(0, 0.4, 0, time)
                        .addStep(0, 0.3, 0, time)
                        .addStep(0, 0.2, 0, time)
                        .addStep(0, 0.1, 0, time)
                        //.addStep(240, 255, 240, time)
                        //.addStep(176, 255, 180, time)
                        //Blanco
                        .addStep(0.1, 0.1, 0.1, time)
                        .addStep(0.2, 0.2, 0.2, time)
                        .addStep(0.3, 0.3, 0.3, time)
                        .addStep(0.4, 0.4, 0.4, time)
                        .addStep(0.5, 0.5, 0.5, time)
                        .addStep(0.6, 0.6, 0.6, time)
                        .addStep(0.7, 0.7, 0.7, time)
                        .addStep(0.8, 0.8, 0.8, time)
                        .addStep(0.9, 0.9, 0.9, time)
                        .addStep(1, 1, 1, time)
                        .addStep(0.9, 0.9, 0.9, time)
                        .addStep(0.8, 0.8, 0.8, time)
                        .addStep(0.7, 0.7, 0.7, time)
                        .addStep(0.6, 0.6, 0.6, time)
                        .addStep(0.5, 0.5, 0.5, time)
                        .addStep(0.4, 0.4, 0.4, time)
                        .addStep(0.3, 0.3, 0.3, time)
                        .addStep(0.2, 0.2, 0.2, time)
                        .addStep(0.1, 0.1, 0.1, time)

                        //Rojo
                        .addStep(0.1, 0, 0, time)
                        .addStep(0.2, 0, 0, time)
                        .addStep(0.3, 0, 0, time)
                        .addStep(0.4, 0, 0, time)
                        .addStep(0.5, 0, 0, time)
                        .addStep(0.6, 0, 0, time)
                        .addStep(0.7, 0, 0, time)
                        .addStep(0.8, 0, 0, time)
                        .addStep(0.9, 0, 0, time)
                        .addStep(1, 0, 0, time)
                        .addStep(0.9, 0, 0, time)
                        .addStep(0.8, 0, 0, time)
                        .addStep(0.7, 0, 0, time)
                        .addStep(0.6, 0, 0, time)
                        .addStep(0.5, 0, 0, time)
                        .addStep(0.4, 0, 0, time)
                        .addStep(0.3, 0, 0, time)
                        .addStep(0.2, 0, 0, time)
                        .addStep(0.1, 0, 0, time)

                        //.addStep(193, 0,0, time)
                        .build();

                timer.start();
                gamepad.runLedEffect(ledEffect);
            }

            @Override
            public boolean isFinished() {
                return timer.done();
            }
        };
    }

    public CommandBase closeLock() {
        return new CommandBase() {
            @Override
            public void initialize() {
                climberTimer.start();
                setClimberLockMode(LockMode.CLOSE);
            }

            @Override
            public void end(boolean interrupted) {
                setClimberMode(ClimberModes.OFF);
            }

            @Override
            public boolean isFinished() {
                return timer.done();
            }
        };
    }

    public CommandBase dpadUpCommand() {
        return new CommandBase() {
            @Override
            public void initialize() {
                if (changeGamepadConfig) {
                    ManualLift().initialize();
                } else {
                    //compressorTrajectory().schedule();
                    automaticClimbingSequence().schedule();
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (changeGamepadConfig)
                    ManualLift().end(true);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase dpadDownCommand() {
        return new CommandBase() {
            @Override
            public void initialize() {
                if (changeGamepadConfig) {
                    ManualRetract().initialize();
                } else {
                    automaticClimb().schedule();
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (changeGamepadConfig)
                    ManualRetract().end(interrupted);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase rightBumperCommand() {
        return new CommandBase() {
            @Override
            public void initialize() {
                changeGamepadConfig = true;
            }

            @Override
            public void end(boolean interrupted) {
                changeGamepadConfig = false;
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }


    public CommandBase intakeCarbons() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setIntakeMode(IntakeMode.INTAKE);
            }

            @Override
            public void end(boolean interrupted) {
                setIntakeMode(IntakeMode.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase rejectCarbons() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setIntakeMode(IntakeMode.REJECT);
            }

            @Override
            public void end(boolean interrupted) {
                setIntakeMode(IntakeMode.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase ManualClimb() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setClimberMode(ClimberModes.CLIMB);
            }

            @Override
            public void end(boolean interrupted) {
                setClimberMode(ClimberModes.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase ManualDown() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setClimberMode(ClimberModes.GO_DOWN);
            }

            @Override
            public void end(boolean interrupted) {
                setClimberMode(ClimberModes.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase ManualLift() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setLinearMode(LinearSystemModes.LIFT_ARM);
            }

            @Override
            public void end(boolean interrupted) {
                setLinearMode(LinearSystemModes.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public CommandBase ManualRetract() {
        return new CommandBase() {
            @Override
            public void initialize() {
                setLinearMode(LinearSystemModes.RETRACT_ARM);
            }

            @Override
            public void end(boolean interrupted) {
                setLinearMode(LinearSystemModes.OFF);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public void cancelAutonomousRoutines() {
        setClimberLockMode(LockMode.OPEN);
        setClimberMode(ClimberModes.OFF);
        setLinearMode(LinearSystemModes.OFF);

        //setToucheMode(ToucheMode.RETRACT);

        setIntakeMode(IntakeMode.OFF);

        setDriveTrajectoryMode(DriveTrajectories.NON_AUTO);
        setDriveMode(DriveModes.CANCEL_AUTONOMOUS_DRIVE);
    }

    public SequentialCommandGroup automaticClimb() {
        return new SequentialCommandGroup(
                new DeadLineStateCommand(toucheSubsystem, ToucheState.RETRACTED.toString(),
                        () -> setToucheMode(ToucheMode.RETRACT)),

                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.FINISHED_TRAJECTORIES.toString(),
                        () -> {
                            setDriveTrajectoryMode(DriveTrajectories.PREPARING_TO_CLIMB);
                            setDriveMode(DriveModes.AUTONOMOUS_DRIVE);
                        }),

                new DeadLineStateCommand(linearSystemSubsystem, LinearSystemStates.IS_RETRACTED.toString(),
                        () -> setLinearMode(LinearSystemModes.AUTOMATIC_RETRACTION)),

                //new InstantCommand(()-> setLinearMode(LinearSystemModes.AUTOMATIC_RETRACTION)),

                new DeadLineStateCommand(climberSubsystem, ClimberState.SAFE_CLIMB.toString(),
                        () -> setClimberMode(ClimberModes.AUTOMATIC_CLIMB)),

                new DeadLineStateCommand(climberLockSubsystem, LockStates.IS_CLOSED.toString(),
                        () -> setClimberLockMode(LockMode.CLOSE))
        );
    }

    public SequentialCommandGroup compressorTrajectory() {
        return new SequentialCommandGroup(
                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.MANUAL_DRIVE.toString(),
                        () -> setDriveMode(DriveModes.RELOCATE_DRIVE_DRIVERS_WALL)),

                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.FINISHED_TRAJECTORIES.toString(),
                        () -> {
                            setDriveTrajectoryMode(DriveTrajectories.SELF_POSITIONING_CLIMBING);
                            setDriveMode(DriveModes.AUTONOMOUS_DRIVE);
                        }));
    }

    public SequentialCommandGroup repositionToCarbon() {
        return new SequentialCommandGroup(
                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.MANUAL_DRIVE.toString(),
                        () -> setDriveMode(DriveModes.RELOCATE_DRIVE_DRIVERS_WALL)),

                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.MANUAL_DRIVE.toString(),
                        () -> {
                            setDriveTrajectoryMode(DriveTrajectories.SELF_POSITIONING_HUMAN_PLAYER);
                            setDriveMode(DriveModes.AUTONOMOUS_DRIVE);
                        }));
    }

    public SequentialCommandGroup automaticClimbingSequence() {
        return new SequentialCommandGroup(
                compressorTrajectory(),
                automaticHookDepose()
        );
    }

    public SequentialCommandGroup automaticHookDepose() {
        return new SequentialCommandGroup(
                new DeadLineStateCommand(toucheSubsystem, ToucheState.IS_POSITIONED_TO_CRASH.toString(),
                        () -> setToucheMode(ToucheMode.TOUCHE_MODE)),
                //new InstantCommand(() -> setToucheMode(ToucheMode.TOUCHE_MODE)),

                new DeadLineStateCommand(tankDriveSubsystem, DriveStates.IS_ALIGNED.toString(),
                        () -> {
                            setDriveTrajectoryMode(DriveTrajectories.ADVANCING_TO_SINK);
                            setDriveMode(DriveModes.AUTONOMOUS_DRIVE);
                        }),

                new DeadLineStateCommand(linearSystemSubsystem, LinearSystemStates.IS_EXTENDED.toString(),
                        () -> {
                            setDriveMode(DriveModes.MANUAL_DRIVE);
                            setLinearMode(LinearSystemModes.AUTOMATIC_ARM_UP);
                        })
        );
    }
}
