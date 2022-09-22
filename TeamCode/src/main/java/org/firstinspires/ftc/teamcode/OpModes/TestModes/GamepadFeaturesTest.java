package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class GamepadFeaturesTest extends ClockMode_V2 {
    private Gamepad.RumbleEffect endGameAnnouncer;    // Use to build a custom rumble sequence.


    @Override
    public void initialize() {

        register();
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(() -> telemetry.addData("Esta", "presionao"));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        return CompetitionStages.TELEOP;
    }

    @Override
    public double getTeleOpTimeOut() {
        return NORMAL_MATCH_TIME;
    }

    @Override
    public Runnable whenEndgame() {
        return () -> {
            endGameAnnouncer = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 1500)
                    .addStep(0.0, 0.0, Gamepad.RUMBLE_DURATION_CONTINUOUS)
                    .build();

            gamepad1.runRumbleEffect(endGameAnnouncer);
        };
    }

}
