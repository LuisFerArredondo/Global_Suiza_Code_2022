package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.Enums.DriveModes;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis.TankDriveSubsystem;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.DriveStates;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

@TeleOp
public class ChassisTeleOp extends ClockMode_V4 {
    private SampleTankDrive sampleTankDrive;
    private TankDriveSubsystem tankDriveSubsystem;
    private GamepadEx gamepadEx1;
    private Vector2d setPoint = new Vector2d(0, -46);

    @Override
    public void initialize() {
        sampleTankDrive = new SampleTankDrive(hardwareMap);
        tankDriveSubsystem = new TankDriveSubsystem(sampleTankDrive);
        gamepadEx1 = new GamepadEx(gamepad1);

        register(tankDriveSubsystem);
        tankDriveSubsystem.setDefaultCommand(new TankDriveCommand(tankDriveSubsystem, () -> -gamepadEx1.getLeftY(), gamepadEx1::getRightX));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.A)
                .toggleWhenActive(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTO_ALIGN_DRIVE),
                        () -> tankDriveSubsystem.setActualMode(DriveModes.MANUAL_DRIVE));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(() -> tankDriveSubsystem.setActualMode(DriveModes.AUTONOMOUS_DRIVE));


        schedule(new RunCommand(() -> {
            //tankDriveSubsystem.update();

            if(tankDriveSubsystem.getActualState().equals(DriveStates.IS_AUTO_ALIGNING))
                tankDriveSubsystem.drawAutoAlign();
            tankDriveSubsystem.drawBot();
        }));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public double getTeleOpTimeOut() {
        return EXPOSITION_TIME;
    }
/*
    @Override
    public void printSubsystemsStates() {
        telemetry.addData("Chassis State", ColorFormatter.ORANGE.format(tankDriveSubsystem.getActualState().toString()));
    }
*/
    @Override
    public @NonNull CompetitionStages setMatchState() {
        return TELEOP;
    }
}
