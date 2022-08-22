package org.firstinspires.ftc.teamcode.Subsystems.Climber.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Climber.ClimberSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Climber.Enums.ClimberModes;
import org.firstinspires.ftc.teamcode.Team15600Lib.Util.JustOnce;

public class ClimberCommand extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private ClimberModes climberModes;
    private boolean flag;
    private JustOnce crash;

    public ClimberCommand(ClimberSubsystem climberSubsystem, ClimberModes climberModes){
        this.climberSubsystem = climberSubsystem;
        this.climberModes = climberModes;
        crash = new JustOnce();

        flag = false;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        if(climberModes != null){
            climberSubsystem.setClimberModes(climberModes);
            climberSubsystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void execute() {
        if(climberModes != null)
            switch (climberModes) {
                case UP:
                    crash.JustOneTime(!flag, ()-> flag = true);

                    climberSubsystem.setMotorsPower(1);
                    break;
                case DOWN:
                    climberSubsystem.setMotorsPower(-1);
                    break;
            }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotorsPower(0);

        climberSubsystem.setClimberModes(ClimberModes.OFF);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
