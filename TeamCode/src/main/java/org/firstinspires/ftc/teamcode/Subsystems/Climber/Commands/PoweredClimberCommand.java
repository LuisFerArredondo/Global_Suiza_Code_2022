package org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;

public class PoweredClimberCommand extends CommandBase {
    private final ClimberSubsystem climberWithPulleySubsystem;
    private ClimberModes climberModes;

    public PoweredClimberCommand(ClimberSubsystem climberWithPulleySubsystem, ClimberModes climberModes){
        this.climberWithPulleySubsystem = climberWithPulleySubsystem;
        this.climberModes = climberModes;

        addRequirements(climberWithPulleySubsystem);
    }

    @Override
    public void initialize() {
        if(climberModes != null){
            climberWithPulleySubsystem.setClimberModes(climberModes);
            climberWithPulleySubsystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void execute() {
        if(climberModes != null)
            switch (climberModes){
                case UP:
                    climberWithPulleySubsystem.setMotorsPower(1);
                    break;
                case DOWN:
                    climberWithPulleySubsystem.setMotorsPower(-1);
                    break;
            }
    }

    @Override
    public void end(boolean interrupted) {
        climberWithPulleySubsystem.setMotorsPower(0);

        climberWithPulleySubsystem.setClimberModes(ClimberModes.OFF);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
