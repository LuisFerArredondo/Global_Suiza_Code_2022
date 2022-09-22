package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.ThreadState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.BlueThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@TeleOp
@Disabled
public class ThreadTestOpMode extends ClockMode_V2 {
    BlueThread we;
    ThreadState threadState = ThreadState.SLEEPING;

    @Override
    public void initialize() {
        register();

        telemetry.addData("Thread State", threadState);
        telemetry.update();

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->{
                    try {
                        we = new BlueThread(false, ()->{
                        threadState = we.getThreadState();
                        });
                        we.start();
                    }catch (Exception ex){
                        telemetry.addData("no jalo we","valio madres");
                        telemetry.update();
                    }
                }));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()->{
                    try {
                    we.interrupt();
                    }catch (Exception ignored){}
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
