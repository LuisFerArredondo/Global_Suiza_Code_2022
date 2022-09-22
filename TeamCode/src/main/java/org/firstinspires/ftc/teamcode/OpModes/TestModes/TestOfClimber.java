package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V4;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

//3.6 : 1
//108 : 30
@TeleOp
@Disabled
public class TestOfClimber extends ClockMode_V4 {
    DcMotorEx climberM;
    Servo servo;
    private final double GearRatio = 7.2;
    private final double TicksPerRev = 288 * GearRatio;//CoreHex ticks per rev = 288
    private final double ticksPerDegree = TicksPerRev / 360;

    @Override
    public void initialize() {
        climberM = hardwareMap.get(DcMotorEx.class, "CM");
        servo = hardwareMap.get(Servo.class,"S");

        climberM.setDirection(DcMotorSimple.Direction.FORWARD);
        climberM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberM.setTargetPositionTolerance(50);

        register();

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RunCommand(()->{
                    climberM.setTargetPosition((int)(ticksPerDegree * 50));
                    climberM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    climberM.setPower(1);

                    if(climberM.getCurrentPosition() > 287.0)servo.setPosition(1);
                }));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new RunCommand(()->{
                    climberM.setTargetPosition(0);
                    climberM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    climberM.setPower(0.8);
                }));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whenPressed(()->servo.setPosition(1));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(()->servo.setPosition(0));


        schedule(new RunCommand(()->{
            telemetry.addData("ticks", climberM.getCurrentPosition());
            telemetry.addData("degree", climberM.getCurrentPosition() / ticksPerDegree);
        }));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        return TELEOP;
    }
}
