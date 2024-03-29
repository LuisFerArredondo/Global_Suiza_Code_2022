package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V6;
import org.firstinspires.ftc.teamcode.Team15600Lib.ClockMode_V7;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

@Config
@Autonomous
public class MotorTester extends ClockMode_V7 {
    public static int TargetTicks = 700;
    private DcMotorEx motorToTest;

    @Override
    public void initialize() {
        motorToTest = hardwareMap.get(DcMotorEx.class, "M");
        motorToTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorToTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        register();

        schedule(new RunCommand(()->{
            setExtraStates("motor ticks", "Ticks: " + motorToTest.getCurrentPosition());
        }));
    }

    @Override
    public Runnable whenTeleOp() {
        return ()->{
          motorToTest.setTargetPosition(TargetTicks);
          motorToTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          motorToTest.setPower(1);
        };
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
