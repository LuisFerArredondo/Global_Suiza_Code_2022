package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.AUTONOMOUS;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V3;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;

@Autonomous
public class TestOfClockOpMode extends ClockMode_V3 {

    @Override
    public void initialize() {

        register();

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A)
                .whileHeld(()->telemetry.addData("webis", "mas"));

        schedule(new RunCommand(()->telemetry.addLine("\n\n\n\n\n\n\n\n")));
        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new ParallelDeadlineGroup(new WaitCommand(3000), new RunCommand(()->{telemetry.addLine("Todo cambió");})),
                        new ParallelDeadlineGroup(new WaitCommand(5700), new RunCommand(()->{telemetry.addLine("Cuando Te vi, oh, oh, oh");})),
                        new ParallelDeadlineGroup(new WaitCommand(1500), new RunCommand(()->{telemetry.addLine("De blanco y negro al ");})),
                        new ParallelDeadlineGroup(new WaitCommand(1000), new RunCommand(()->{telemetry.addLine(ColorFormatter.CYAN.format("COLOR"));})),
                        new ParallelDeadlineGroup(new WaitCommand(3000), new RunCommand(()->{telemetry.addLine(ColorFormatter.LIME.format("Me convertí"));}))
                        //segundo 18 - 29 9 seg
                ));
    }

    @Override
    public VisionThread VisionThread() {
        return null;
    }

    @Override
    public @NonNull CompetitionStages setMatchState() {
        return AUTONOMOUS;
    }

    @Override
    public void printSubsystemsStates() {

    }

}
