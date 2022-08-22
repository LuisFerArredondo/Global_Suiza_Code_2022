package org.firstinspires.ftc.teamcode.Team15600Lib;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.ENDGAME;
import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.INITIALIZING;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem;

public abstract class ClockMode extends LinearOpMode {
    private ElapsedTime matchTimer = new ElapsedTime();
    private CompetitionStages competitionStages = INITIALIZING;

    // TeleOp timings
    public static final double NORMAL_MATCH_TIME = 90;// duration in seconds
    public static final double GLOBAL_MATCH_TIME = 120;// duration in seconds
    public static final double EXPOSITION_TIME = -1;// never reaching setpoint

    private double TimeForTeleOp;

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    /**
     * Schedules {@link com.arcrobotics.ftclib.command.Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link com.arcrobotics.ftclib.command.Subsystem} objects to the scheduler
     */
    public void register(BrickSystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Match Stage:", competitionStages.toString());
        telemetry.addData("Vision state", "initializing");

        TimeForTeleOp = getTeleOpTimeOut();

        try {
            VisionThread().start();
        }catch (Exception ex){
            telemetry.addData("Vision state", "missing thread");
        }

        telemetry.update();
        initialize();

        //Waits the OpMode to start<hahaha>
        waitForStart();
        telemetry.setMsTransmissionInterval(10);

        try {
            VisionThread().interrupt();
        }catch (Exception ignored){}

        competitionStages = setMatchState();
        clearTelemetry();
        matchTimer.reset();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Match Stage:", competitionStages.toString());

            switch (competitionStages){
                case AUTONOMOUS:
                    WhenAutonomous();
                    break;

                case TELEOP:
                    WhenTeleOp();

                    if(matchTimer.seconds() >= TimeForTeleOp
                            && TimeForTeleOp != EXPOSITION_TIME)
                        competitionStages = ENDGAME;
                    break;

                case ENDGAME:
                    WhenEndgame();
                    break;
            }

            run();
        }
        reset();
    }

    //Note: Register the subsystems even do u are not using commands
    public abstract void initialize();

    //Note: Command Scheduler is working, commands can also be used in these
    public void WhenAutonomous(){}
    public void WhenTeleOp(){}
    public void WhenEndgame(){}

    /**
     * Note: create a class that implements {@link Runnable} interface
     * to implement vision
     *
     * Not obligatory required
     * */
    public abstract VisionThread VisionThread();

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }

    public void clearTelemetry() {
        telemetry.clearAll();
        telemetry.update();
    }


    public abstract @NonNull CompetitionStages setMatchState();
    public abstract double getTeleOpTimeOut();

    public CompetitionStages getCompetitionMatchStages(){
        return competitionStages;
    }
}
