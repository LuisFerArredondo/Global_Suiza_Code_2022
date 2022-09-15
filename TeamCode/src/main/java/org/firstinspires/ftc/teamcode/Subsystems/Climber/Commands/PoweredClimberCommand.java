package org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

public class PoweredClimberCommand extends CommandBase {
    private final ClimberSubsystem climberWithPulleySubsystem;
    private JustOnce justOnce;

    public PoweredClimberCommand(ClimberSubsystem climberWithPulleySubsystem) {
        this.climberWithPulleySubsystem = climberWithPulleySubsystem;
        justOnce = new JustOnce();

        addRequirements(climberWithPulleySubsystem);
    }

    @Override
    public void initialize() {
        climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void execute() {
        switch (climberWithPulleySubsystem.getActualMode()) {
            case CLIMB:
                //justOnce.JustOneTime(true, () -> justOnce.initTimer(500));
                //climberWithPulleySubsystem.setLockPosition(1);

                //if (justOnce.getTimerDone())
                    climberWithPulleySubsystem.setClimberPower(1);
                climberWithPulleySubsystem.setActualState(ClimberState.CLIMBING);
                break;
            case GO_DOWN:
                //justOnce.JustOneTime(true, () -> justOnce.initTimer(500));
                //climberWithPulleySubsystem.setLockPosition(1);

                //if (justOnce.getTimerDone())
                    climberWithPulleySubsystem.setClimberPower(-1);
                climberWithPulleySubsystem.setActualState(ClimberState.GOING_DOWN);
                break;
            case LIFT_ARM:
                climberWithPulleySubsystem.setLinearSystemPower(1);
                climberWithPulleySubsystem.setActualState(ClimberState.DEPOSING_HOOK);
                break;
            case ARM_UP:
                break;
            case RETRACT_ARM:
                climberWithPulleySubsystem.setLinearSystemPower(-1);
                climberWithPulleySubsystem.setActualState(ClimberState.IN_RETRACTING);
                break;
            case OFF:
                //climberWithPulleySubsystem.setLockPosition(0.55);
                climberWithPulleySubsystem.setClimberPower(0);
                climberWithPulleySubsystem.setLinearSystemPower(0);
                climberWithPulleySubsystem.setActualState(ClimberState.IDLE);
                break;

        }
    }

    @Override
    public void end(boolean interrupted) {
        climberWithPulleySubsystem.setClimberPower(0);
        climberWithPulleySubsystem.setLinearSystemPower(0);

        climberWithPulleySubsystem.setActualMode(ClimberModes.OFF);
        climberWithPulleySubsystem.setActualState(ClimberState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
