package org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;

public class ToucheCommand extends CommandBase {
    private ToucheSubsystem toucheSubsystem;
    private int targetPos = 0;

    public ToucheCommand(ToucheSubsystem toucheSubsystem) {
        this.toucheSubsystem = toucheSubsystem;

        addRequirements(toucheSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (toucheSubsystem.getActualMode()){
            case TOUCHE_MODE:
                targetPos = 0;
                toucheSubsystem.setLeftArmTicks(targetPos);
                toucheSubsystem.setLeftArmPower(1);

                toucheSubsystem.setRightArmTicks(targetPos);
                toucheSubsystem.setRightArmPower(1);

                if(!toucheSubsystem.areMotorsBusy())
                toucheSubsystem.setActualState(ToucheState.POSITIONED_TO_CRASH);
                else
                    toucheSubsystem.setActualState(ToucheState.POSITIONING_TO_CRASH);
                break;

            case RETRACT:
                targetPos = 0;
                toucheSubsystem.setLeftArmTicks(targetPos);
                toucheSubsystem.setLeftArmPower(1);

                toucheSubsystem.setRightArmTicks(targetPos);
                toucheSubsystem.setRightArmPower(1);

                if(!toucheSubsystem.areMotorsBusy())
                    toucheSubsystem.setActualState(ToucheState.RETRACTED);
                else
                    toucheSubsystem.setActualState(ToucheState.RETRACTING);
                break;

            case CLIMB_MODE:
                targetPos = 0;
                toucheSubsystem.setLeftArmTicks(targetPos);
                toucheSubsystem.setLeftArmPower(1);

                toucheSubsystem.setRightArmTicks(targetPos);
                toucheSubsystem.setRightArmPower(1);

                if(!toucheSubsystem.areMotorsBusy())
                    toucheSubsystem.setActualState(ToucheState.CLIMB_IS_CORRECTED);
                else
                    toucheSubsystem.setActualState(ToucheState.IS_CORRECTING_CLIMB);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
