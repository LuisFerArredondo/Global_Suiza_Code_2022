package org.firstinspires.ftc.teamcode.Subsystems.Intake.Commands;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Enums.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private boolean isFilled = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (intakeSubsystem.getActualMode()) {
            case OFF:
                if (isFilled)
                    intakeSubsystem.setActualState(IntakeState.IS_FILLED);
                else
                    intakeSubsystem.setActualState(IntakeState.IDLE);
                intakeSubsystem.setIntakePower(0);
                //intakeSubsystem.setBlenderPower(0);
                break;
            case INTAKE:
                if (isFilled) {
                    intakeSubsystem.setActualMode(IntakeMode.OFF);
                    break;
                }
                if(intakeSubsystem.getMotorCurrent() > 7.6 ){
                    isFilled = true;
                }

                intakeSubsystem.setActualState(IntakeState.IS_PICKING);
                intakeSubsystem.setIntakePower(1);
                //intakeSubsystem.setBlenderPower(1);
                break;
            case REJECT:
                intakeSubsystem.setActualState(IntakeState.IS_REJECTING_AND_BLENDING);
                intakeSubsystem.setIntakePower(-1);
                //intakeSubsystem.setBlenderPower(1);
                isFilled = false;
                break;
        }

        packet.put("intake Motor current", intakeSubsystem.getMotorCurrent());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setActualState(IntakeState.IDLE);
        intakeSubsystem.setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
