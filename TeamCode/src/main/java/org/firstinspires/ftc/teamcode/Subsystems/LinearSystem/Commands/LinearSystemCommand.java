package org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.ClimberLockSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimberLock.Enums.LockStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemModes;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.Enums.LinearSystemStates;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSystem.LinearSystemSubsystem;

public class LinearSystemCommand extends CommandBase {
    private final LinearSystemSubsystem linearSystem;

    public LinearSystemCommand(LinearSystemSubsystem climberWithPulleySubsystem) {
        this.linearSystem = climberWithPulleySubsystem;

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
                linearSystem.setLinearSystemMotorTicks(4522);
                linearSystem.setLinearSystemPower(1);
                linearSystem.setActualState(LinearSystemStates.DEPOSING_HOOK);
                break;

            case AUTOMATIC_RETRACTION:
                linearSystem.setLinearSystemMotorTicks(0);
                linearSystem.setLinearSystemPower(1);
                linearSystem.setActualState(LinearSystemStates.IS_RETRACTING);
                break;

            /** OFF Everything*/
            case OFF:
                linearSystem.setLinearSystemPower(0);
                linearSystem.setActualState(LinearSystemStates.IDLE);
                break;
        }
        //FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

