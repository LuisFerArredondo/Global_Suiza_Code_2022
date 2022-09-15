package org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands;

import static org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem_V2;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberState;

public class ClimberCommand extends CommandBase {
    private final ClimberSubsystem_V2 climberSubsystem;
    private final double GEAR_RATIO = 17.3;
    private final double TicksPerRev = 125;
    private final double TicksPerDegree = (GEAR_RATIO * TicksPerRev) / 360;

    public ClimberCommand(ClimberSubsystem_V2 climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setActualMode(ClimberModes.INIT_POSITION);
    }

    @Override
    public void execute() {
        switch (climberSubsystem.getActualMode()) {
            case ARM_UP:
                climberSubsystem.setActualState(IS_EXTENDING);
                climberSubsystem.setClimberArmTicks((int) (35 * TicksPerDegree));
                climberSubsystem.setClimberArmPower(1);

                // if(climberSubsystem.getClimberArmTicks() > (40 * TicksPerDegree)){
                //     climberSubsystem.setActualMode(ClimberModes.LIFT_ARM);
                //}
                break;

            case RETRACT_ARM:
                climberSubsystem.setActualState(IN_RETRACTING);

                climberSubsystem.setClimberArmTicks(0);
                climberSubsystem.setClimberArmPower(1);
                break;

            case CRASHING_POSITION:
                climberSubsystem.setActualState(IN_POSSIBLE_CRASHING);
                climberSubsystem.setClimberStabilizerPos(0.6);
                break;

            case LIFT_ARM:
                climberSubsystem.setActualState(DEPOSING_HOOK);
                climberSubsystem.setClimberStabilizerPos(1);
                break;
            case INIT_POSITION:
                climberSubsystem.setActualState(RESETTING_POSITIONS);
                climberSubsystem.setClimberStabilizerPos(0);
                break;

            case CLIMB:
                climberSubsystem.setActualState(CLIMBING);
                climberSubsystem.setClimberClimberMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climberSubsystem.setClimberClimberPower(1);
                break;

            case GO_DOWN:
                climberSubsystem.setActualState(GOING_DOWN);
                climberSubsystem.setClimberClimberMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climberSubsystem.setClimberClimberPower(-1);
                break;

            case OFF:
                climberSubsystem.setActualState(IDLE);
                climberSubsystem.setClimberArmPower(0);
                climberSubsystem.setClimberClimberPower(0);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
