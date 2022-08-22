package com.Team15600.meepmeep.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestTrajectory {
    public static RoadRunnerBotEntity traj (MeepMeep meepMeep){
        return new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(12,13)
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(20, 30, Math.toRadians(180)))
                                .setReversed(drive.getPoseEstimate().getHeading() >= 0 && drive.getPoseEstimate().getHeading() <= 225)
                                .splineTo(new Vector2d(0,-46),Math.toRadians(-90))

                                .build()
                );
    }

    public boolean getRobotReverseDirection(Pose2d robotPose){
       // if()
        return false;
    }
}
