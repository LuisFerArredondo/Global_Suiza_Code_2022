package org.firstinspires.ftc.teamcode.Team15600Lib;

import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.AUTONOMOUS;
import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.ENDGAME;
import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.INITIALIZING;
import static org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages.TELEOP;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.CompetitionStages;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.BrickSystem_V2;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.ColorFormatter;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public abstract class ClockMode_V4 extends LinearOpMode {
    private final ElapsedTime matchTimer = new ElapsedTime();
    private CompetitionStages competitionStages = INITIALIZING;

    private static final String VISION_TAG = "Vision Thread State ";
    private static final String AUTONOMOUS_TAG = "Autonomous Thread State ";
    private static final String TELEOP_TAG = "TeleOp Thread State ";
    private static final String ENDGAME_TAG = "EndGame Thread State ";

    private final List<BrickSystem_V2> m_subsystems = new LinkedList<>();


    private Thread AutoThread;
    private Thread TeleOpThread;
    private Thread EndgameThread;

    // TeleOp timings
    public static final double NORMAL_MATCH_TIME = 90;// duration in seconds
    public static final double GLOBAL_MATCH_TIME = 120;// duration in seconds
    public static final double EXPOSITION_TIME = -1;// never reaching setpoint

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
    public void register(BrickSystem_V2... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
        m_subsystems.addAll(Arrays.asList(subsystems));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /***************** Initializing OpMode **************************/
        //if (!this.getClass().isAnnotationPresent(Autonomous.class))
        //   telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setCaptionValueSeparator(" ‖ ");// ⁞, …, ¶
        telemetry.addData("Match Stage:", competitionStages.toString());
        telemetry.addData("Vision state", ColorFormatter.YELLOW.format("initializing"));
        RobotLog.dd(VISION_TAG, "on init");
        //telemetry.update();

        double timeForTeleOp = getTeleOpTimeOut();


        JustOnce whenAutonomous = new JustOnce();
        JustOnce whenTeleOp = new JustOnce();
        JustOnce whenEndgame = new JustOnce();

        try {
            VisionThread().start();
            RobotLog.dd(VISION_TAG, " thread Successfully enabled");
        } catch (Exception ex) {
            telemetry.addData("Vision state", ColorFormatter.RED.format("missing thread"));
            RobotLog.dd(VISION_TAG, "Missing Thread");
        }

        telemetry.update();
        initialize();

        /***************** OpMode Started **************************/
        waitForStart();
        telemetry.setMsTransmissionInterval(10);

        try {
            VisionThread().interrupt();
            RobotLog.dd(VISION_TAG, "Thread successfully finished");
        } catch (Exception ignored) {
        }

        competitionStages = setMatchState();
        clearTelemetry();
        matchTimer.reset();


        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            // if (this.getClass().isAnnotationPresent(Autonomous.class))
            //else
            telemetry.addData("Match Stage:", competitionStages.equals(AUTONOMOUS) ?
                    ColorFormatter.GREEN.format(competitionStages.toString()) :
                    competitionStages.equals(TELEOP) ?
                            ColorFormatter.LIME.format(competitionStages.toString()) :
                            ColorFormatter.CYAN.format(competitionStages.toString()));

            telemetry.addData("Timer Counts", ColorFormatter.YELLOW.format(String.valueOf(matchTimer.seconds())) + "\n");
            switch (competitionStages) {
                case AUTONOMOUS:
                    whenAutonomous.JustOneTime(true, () -> {
                        try {
                            AutoThread = new Thread(whenAutonomous());
                            AutoThread.start();
                            RobotLog.dd(AUTONOMOUS_TAG, "Successfully enabled");
                        } catch (Exception ignored) {
                        }
                    });
                    break;

                case TELEOP:
                    whenTeleOp.JustOneTime(true, () -> {
                        try {
                            TeleOpThread = new Thread(whenTeleOp());
                            TeleOpThread.start();
                            RobotLog.dd(TELEOP_TAG, "Successfully enabled");
                        } catch (Exception ignored) {
                        }
                    });

                    if (matchTimer.seconds() >= timeForTeleOp
                            && timeForTeleOp != EXPOSITION_TIME)
                        competitionStages = ENDGAME;
                    break;

                case ENDGAME:
                    whenEndgame.JustOneTime(true, () -> {
                        try {
                            EndgameThread = new Thread(whenEndgame());
                            EndgameThread.start();
                            RobotLog.dd(ENDGAME_TAG, "Successfully enabled");
                        } catch (Exception ignored) {
                        }
                    });
                    break;
            }

            printSubsystemsStates();
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
                RobotLog.dd(AUTONOMOUS_TAG, "successfully finished");
            } catch (Exception ignored) {
            }
    }

    public void cancelTeleOpThread() {
        if (TeleOpThread != null)
            try {
                TeleOpThread.interrupt();
                RobotLog.dd(TELEOP_TAG, "successfully finished");
            } catch (Exception ignored) {
            }
    }

    public void cancelEndgameThread() {
        if (EndgameThread != null)
            try {
                EndgameThread.interrupt();
                RobotLog.dd(ENDGAME_TAG, "successfully finished");
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

    public double getTeleOpTimeOut() {
        return NORMAL_MATCH_TIME;
    }

    public CompetitionStages getCompetitionMatchStages() {
        return competitionStages;
    }

    public void printSubsystemsStates() {
        for (BrickSystem_V2 subsystem : m_subsystems) {
            telemetry.addData(subsystem.getName() + " State"
                    , subsystem.getSelectedTelemetryColor().format(subsystem.getSubsystemState()));
        }
    }

}