package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V5;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;

@TeleOp
@Disabled
public class IntakeTeleOp extends ClockMode_V6 {
    SampleTankDrive sampleTankDrive;
    TankDriveSubsystem tankDriveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ToucheSubsystem toucheSubsystem;
    GamepadEx gamepadEx1;

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive, hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        //toucheSubsystem = new ToucheSubsystem(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);

        register(tankDriveSubsystem, intakeSubsystem);

        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> -gamepadEx1.getLeftY(), gamepadEx1::getRightX));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(()-> intakeSubsystem.setActualMode(IntakeMode.INTAKE))
                .whenReleased(()-> intakeSubsystem.setActualMode(IntakeMode.OFF));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()-> intakeSubsystem.setActualMode(IntakeMode.REJECT))
                .whenReleased(()-> intakeSubsystem.setActualMode(IntakeMode.OFF));
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
