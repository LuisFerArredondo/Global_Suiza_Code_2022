package org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockMode;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;

@Config
public class LinearSystemCommand extends CommandBase {
    private final LinearSystemSubsystem linearSystem;
    private ClimberSubsystem climberSubsystem;
    private ClimberLockSubsystem climberLockSubsystem;
    public static boolean show_data_2_dash = false;
    private int targetPos = 4757;
    double lastTick = 0;


    public LinearSystemCommand(LinearSystemSubsystem climberWithPulleySubsystem) {
        this.linearSystem = climberWithPulleySubsystem;

        addRequirements(climberWithPulleySubsystem);
    }

    public LinearSystemCommand(LinearSystemSubsystem climberWithPulleySubsystem, ClimberSubsystem climberSubsystem, ClimberLockSubsystem climberLockSubsystem) {
        this(climberWithPulleySubsystem);
        this.climberSubsystem = climberSubsystem;
        this.climberLockSubsystem = climberLockSubsystem;

        addRequirements(climberWithPulleySubsystem);
    }

    @Override
    public void initialize() {
        //climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (linearSystem.getActualMode()) {
            /**Linear System Manual Actions*/
            case LIFT_ARM:
                linearSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linearSystem.setLinearSystemPower(1);
                linearSystem.setActualState(LinearSystemStates.DEPOSING_HOOK);
                break;
            case RETRACT_ARM:
                linearSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linearSystem.setLinearSystemPower(-1);
                linearSystem.setActualState(LinearSystemStates.IS_RETRACTING);
                break;

            /**Linear System Automatic Actions*/
            case AUTOMATIC_ARM_UP:
                lastTick = linearSystem.getLinearSystemMotorTicks();
                linearSystem.setLinearSystemMotorTicks(targetPos);
                linearSystem.setLinearSystemPower(1);

                if (!linearSystem.isMotorBusy() && lastTick != linearSystem.getLinearSystemMotorTicks()) {
                    linearSystem.setActualState(LinearSystemStates.IS_EXTENDED);
                    linearSystem.setActualMode(LinearSystemModes.OFF);
                } else
                    linearSystem.setActualState(LinearSystemStates.IS_EXTENDING);
                break;

            case AUTOMATIC_RETRACTION:
                lastTick = linearSystem.getLinearSystemMotorTicks();
                linearSystem.setLinearSystemMotorTicks(0);
                linearSystem.setLinearSystemPower(1);

                //if(linearSystem.getLinearSystemMotorTicks() < (double)(targetPos * 2) / 3)
                //    linearSystem.setActualMode(LinearSystemModes.AUTOMATIC_CLIMB);
                //if (!linearSystem.isMotorBusy() && lastTick != linearSystem.getLinearSystemMotorTicks()) {
                if (!linearSystem.isMotorBusy() && lastTick != linearSystem.getLinearSystemMotorTicks()) {
                    linearSystem.setActualMode(LinearSystemModes.OFF);
                    //linearSystem.setActualMode(LinearSystemModes.OFF);
                } else if (linearSystem.getLinearSystemMotorTicks() < (((double) targetPos / 4) * 3)) {
                    linearSystem.setActualState(LinearSystemStates.IS_RETRACTED);
                    //justOnce = true;

                } else {
                    linearSystem.setActualState(LinearSystemStates.IS_RETRACTING);
                    //justOnce = false;
                }
                break;

            /*OFF Everything///
            case AUTOMATIC_CLIMB:
                climberSubsystem.setActualMode(ClimberModes.AUTOMATIC_CLIMB);

                if (!climberSubsystem.isMotorBusy())
                    linearSystem.setActualMode(LinearSystemModes.AUTOMATIC_LOCK);
                break;
            case AUTOMATIC_LOCK:
                climberLockSubsystem.setActualMode(LockMode.CLOSE);
                climberSubsystem.setActualMode(ClimberModes.OFF);
                break;
                */
            case OFF:
                linearSystem.setLinearSystemPower(0);
                linearSystem.setActualState(LinearSystemStates.IDLE);
                break;
        }
        packet.put("Linear system Ticks", "Linear system Ticks " + linearSystem.getLinearSystemMotorTicks());
        if (show_data_2_dash)
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
        linearSystem.setLinearSystemPower(0);
        linearSystem.setActualMode(LinearSystemModes.OFF);
        linearSystem.setActualState(LinearSystemStates.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

