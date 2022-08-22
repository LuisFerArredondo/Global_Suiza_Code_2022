package org.firstinspires.ftc.teamcode.OpModes.TestModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous
public class PortsInfoTest extends LinearOpMode {
    /*
    * For a data that samir needed
    * this class is no longer useful
    * */

    //TestSubsystem testSubsystem;
    DcMotorEx motor1;


    @Override
    public void runOpMode() throws InterruptedException {
        //testSubsystem = new TestSubsystem(hardwareMap);
        motor1 = hardwareMap.get(DcMotorEx.class, "M1");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (!isStopRequested()){//dios estoy muy fregado
            motor1.setTargetPosition(288);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.5);

           telemetry.addData("Current motor ", motor1.getCurrent(CurrentUnit.MILLIAMPS));
           telemetry.addData("Current motor position ", motor1.getCurrentPosition());
           telemetry.update();
        }
    }
}
