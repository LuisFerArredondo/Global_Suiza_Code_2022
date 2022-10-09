package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@Config
@Autonomous
public class GamepadColorIdentifier extends ClockMode_V7 {
    public static double r = 0;
    public static double g = 0;
    public static double b = 0;
   
    @Override
    public void initialize() {
        register();

    }

    @Override
    public Runnable whenTeleOp() {
        return () -> {
            while (!Thread.currentThread().isInterrupted()) {
                gamepad1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
            }
        };
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
