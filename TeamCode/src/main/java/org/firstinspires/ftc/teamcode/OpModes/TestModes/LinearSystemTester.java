package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.GamepadButtonMix;

@Config
@Autonomous
public class LinearSystemTester extends ClockMode_V7 {
    public static boolean manual_tester = true;
    public static int TargetTicks = 0;
    private DcMotorEx motorToTest;

    @Override
    public void initialize() {
        motorToTest = hardwareMap.get(DcMotorEx.class, "M");
        motorToTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (manual_tester)
            motorToTest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        else
            motorToTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        register();

        if (manual_tester) {
            new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                    .whileHeld(() -> motorToTest.setPower(1))
                    .whenReleased(() -> motorToTest.setPower(0));

            new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                    .whileHeld(() -> motorToTest.setPower(-1))
                    .whenReleased(() -> motorToTest.setPower(0));

        } else {
            new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                    .whenPressed(() -> {
                        motorToTest.setTargetPosition(TargetTicks);
                        motorToTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorToTest.setPower(1);
                    });

            new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(() -> {
                        motorToTest.setTargetPosition(0);
                        motorToTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorToTest.setPower(1);
                    });
        }

        schedule(new RunCommand(() -> {
            TelemetryPacket telemetryPacket = new TelemetryPacket();

            setExtraStates("motor ticks", "Ticks: " + motorToTest.getCurrentPosition());

            packet.put("motor ticks", "Ticks: " + motorToTest.getCurrentPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        }));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        enablePrintExtraStates();
        setChangeForEndGameTime(EXPOSITION_TIME);
        return CompetitionStages.TELEOP;
    }
}

