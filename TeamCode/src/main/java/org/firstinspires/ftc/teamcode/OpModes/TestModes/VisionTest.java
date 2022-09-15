package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.TensorFlowThreadExample;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

public class VisionTest extends ClockMode_V2 {
    TensorFlowThreadExample tensorFlowThreadExample;

    @Override
    public void initialize() {
        register();
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(()->telemetry.addLine("working"));
    }

    @Override
    public VisionThread VisionThread() {
        tensorFlowThreadExample = new TensorFlowThreadExample(hardwareMap, telemetry);

        return tensorFlowThreadExample;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        return TELEOP;
    }

    @Override
    public double getTeleOpTimeOut() {
        return EXPOSITION_TIME;
    }
}
