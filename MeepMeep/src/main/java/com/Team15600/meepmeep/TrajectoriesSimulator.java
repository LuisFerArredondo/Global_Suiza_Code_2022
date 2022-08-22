package com.Team15600.meepmeep;

import com.Team15600.meepmeep.Trajectories.TestTrajectory;
import com.noahbres.meepmeep.MeepMeep;
public class TrajectoriesSimulator {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(TestTrajectory.traj(meepMeep))
                //.addEntity(mySecondBot)
                .start();
    }
}