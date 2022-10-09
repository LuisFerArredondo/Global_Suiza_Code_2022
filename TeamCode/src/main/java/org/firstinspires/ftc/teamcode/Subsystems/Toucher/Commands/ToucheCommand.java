package org.firstinspires.ftc.teamcode.Subsystems.Toucher.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Toucher.Enums.ToucheState;
import org.firstinspires.ftc.teamcode.Subsystems.Toucher.ToucheSubsystem;

@Config
public class ToucheCommand extends CommandBase {
    public static double Left_Servo_Pos = 0.63;
    public static double Right_Servo_Pos = 0.63;//Este wey esta mamastrozo, nunca se ha roto

    private ToucheSubsystem toucheSubsystem;

    public ToucheCommand(ToucheSubsystem toucheSubsystem) {
        this.toucheSubsystem = toucheSubsystem;

        addRequirements(toucheSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        switch (toucheSubsystem.getActualMode()) {
            case TOUCHE_MODE:
                toucheSubsystem.setLeftArmPosition(Left_Servo_Pos);
                toucheSubsystem.setRightArmPosition(Right_Servo_Pos);

                if(toucheSubsystem.isLeftDetecting() && toucheSubsystem.isRightDetecting())
                    toucheSubsystem.setActualState(ToucheState.IS_ROBOT_POSITIONED);

                else if(toucheSubsystem.isLeftDetecting())
                    toucheSubsystem.setActualState(ToucheState.IS_LEFT_POSITIONED);

                else if(toucheSubsystem.isRightDetecting())
                    toucheSubsystem.setActualState(ToucheState.IS_RIGHT_POSITIONED);

                else
                    toucheSubsystem.setActualState(ToucheState.IS_POSITIONED_TO_CRASH);
                break;

            case RETRACT:
                toucheSubsystem.setRightArmPosition(0.3);
                toucheSubsystem.setLeftArmPosition(0.3);

                toucheSubsystem.setActualState(ToucheState.RETRACTED);
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
