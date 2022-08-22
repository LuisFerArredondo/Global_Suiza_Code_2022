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
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

public abstract class ClockMode_V2 extends LinearOpMode {
    private ElapsedTime matchTimer = new ElapsedTime();
    private CompetitionStages competitionStages = INITIALIZING;
    private JustOnce whenAutonomous;
    private JustOnce whenTeleOp;
    private JustOnce whenEndgame;

    private Thread AutoThread;
    private Thread TeleOpThread;
    private Thread EndgameThread;

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
     * Schedules {@link Command} objects to the scheduler
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
        whenAutonomous = new JustOnce();
        whenTeleOp = new JustOnce();
        whenEndgame = new JustOnce();

        try {
            VisionThread().start();
        } catch (Exception ex) {
            telemetry.addData("Vision state", "missing thread");
        }

        telemetry.update();
        initialize();

        //Waits the OpMode to start<hahaha>
        waitForStart();
        telemetry.setMsTransmissionInterval(10);

        try {
            VisionThread().interrupt();
        } catch (Exception ignored) {
        }

        competitionStages = setMatchState();
        clearTelemetry();
        matchTimer.reset();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Match Stage:", competitionStages.toString());
            telemetry.addData("Timer Counts", matchTimer.seconds());

            switch (competitionStages) {
                case AUTONOMOUS:
                    whenAutonomous.JustOneTime(true, () -> {
                        try {
                            AutoThread = new Thread(whenAutonomous());
                            AutoThread.start();
                            //telemetry.addLine("Auto thread Succsefully eneabled");
                        } catch (Exception ignored) {
                        }
                    });
                    break;

                case TELEOP:
                    whenTeleOp.JustOneTime(true, () -> {
                        try {
                            TeleOpThread = new Thread(whenTeleOp());
                            TeleOpThread.start();
                            telemetry.addLine("teleOp thread Succsefully eneabled");
                        } catch (Exception ignored) {
                        }
                    });

                    if (matchTimer.seconds() >= TimeForTeleOp
                            && TimeForTeleOp != EXPOSITION_TIME)
                        competitionStages = ENDGAME;
                    break;

                case ENDGAME:
                    whenEndgame.JustOneTime(true, () -> {
                        try {
                            EndgameThread = new Thread(whenEndgame());
                            EndgameThread.start();
                            telemetry.addLine("EndGame thread Succsefully eneabled");
                        } catch (Exception ignored) {
                        }
                    });
                    break;
            }

            run();
        }
        clearTelemetry();

        cancelAutoThread();
        cancelTeleOpThread();
        cancelEndgameThread();
        telemetry.update();

        reset();
    }

    //Note: Register the subsystems even do u are not using commands
    public abstract void initialize();

    //Note: Command Scheduler is working, commands can also be used in these
    public Runnable whenAutonomous() {
        return null;
    }

    public Runnable whenTeleOp() {
        return null;
    }

    public Runnable whenEndgame() {
        return null;
    }

    public void cancelAutoThread() {
        if (AutoThread != null)
            try {
                AutoThread.interrupt();
                telemetry.addLine("Auto thread succesfully finished");
            } catch (Exception ignored) {
            }
    }

    public void cancelTeleOpThread() {
        if (TeleOpThread != null)
            try {
                TeleOpThread.interrupt();
                telemetry.addLine("TeleOp thread succesfully finished");
            } catch (Exception ignored) {
            }
    }

    public void cancelEndgameThread() {
        if (EndgameThread != null)
            try {
                EndgameThread.interrupt();
                telemetry.addLine("Endgame thread succesfully finished");
            } catch (Exception ignored) {
            }
    }

    /**
     * Note: create a class that implements {@link Runnable} interface
     * to implement vision
     * <p>
     * Not obligatory required
     */
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

    public double getTeleOpTimeOut(){
        return NORMAL_MATCH_TIME;
    }

    public CompetitionStages getCompetitionMatchStages() {
        return competitionStages;
    }
}
