package org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;

@Config
public class PoweredClimberCommand extends CommandBase {
    private final ClimberSubsystem climberWithPulleySubsystem;
    public static boolean show_data_2_dash = false;
    private double lastTicks = 0;

    public PoweredClimberCommand(ClimberSubsystem climberWithPulleySubsystem) {
        this.climberWithPulleySubsystem = climberWithPulleySubsystem;

        addRequirements(climberWithPulleySubsystem);
    }

    @Override
    public void initialize() {
        //climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (climberWithPulleySubsystem.getActualMode()) {
            /**Climber Manual Actions*/
            case CLIMB:
                if (ClimberLockSubsystem.getActualState().equals(LockStates.IS_CLOSED)) {
                    climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
                    break;
                }

                climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climberWithPulleySubsystem.setClimberPower(1);
                climberWithPulleySubsystem.setActualState(ClimberState.CLIMBING);
                break;

            case GO_DOWN:
                if (ClimberLockSubsystem.getActualState().equals(LockStates.IS_CLOSED)) {
                    climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
                    break;
                }

                climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climberWithPulleySubsystem.setClimberPower(-1);
                climberWithPulleySubsystem.setActualState(ClimberState.GOING_DOWN);
                break;

            /** OFF Everything*/
            case AUTOMATIC_CLIMB:
                if (ClimberLockSubsystem.getActualState().equals(LockStates.IS_CLOSED)) {
                    climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
                    break;
                }
                lastTicks = climberWithPulleySubsystem.getClimberMotorTicks();

                climberWithPulleySubsystem.setClimberMotorTicks(9200);
                climberWithPulleySubsystem.setClimberPower(1);

                if (!climberWithPulleySubsystem.isMotorBusy() && lastTicks != climberWithPulleySubsystem.getClimberMotorTicks())
                    climberWithPulleySubsystem.setActualState(ClimberState.SAFE_CLIMB);
                else
                    climberWithPulleySubsystem.setActualState(ClimberState.IN_POSSIBLE_CRASHING);

                break;
            case AUTO_RELEASE:
                if (ClimberLockSubsystem.getActualState().equals(LockStates.IS_CLOSED)) {
                    climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
                    break;
                }
                //justOnce.JustOneTime(true, () -> justOnce.initTimer(500));
                //climberWithPulleySubsystem.setLockPosition(1);
                //if (justOnce.getTimerDone())
                climberWithPulleySubsystem.setClimberMotorTicks(0);
                climberWithPulleySubsystem.setClimberPower(1);
                climberWithPulleySubsystem.setActualState(ClimberState.IN_RETRACTING);
                break;
            case OFF:
                //climberWithPulleySubsystem.setLockPosition(0.55);
                climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climberWithPulleySubsystem.setClimberPower(0);
                climberWithPulleySubsystem.setActualState(ClimberState.IDLE);
                break;

        }

        packet.put("Climber Consumed Current", "ClimberTicks: " + climberWithPulleySubsystem.getClimberMotorTicks());
        if (show_data_2_dash)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
        climberWithPulleySubsystem.setClimberPower(0);

        climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
        climberWithPulleySubsystem.setActualState(ClimberState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
