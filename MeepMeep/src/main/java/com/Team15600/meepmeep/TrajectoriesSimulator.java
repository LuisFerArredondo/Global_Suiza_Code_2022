package com.Team15600.meepmeep;

import com.Team15600.meepmeep.Trajectories.TestTrajectory;
import com.noahbres.meepmeep.MeepMeep;

import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.ImageObserver;
import java.awt.image.ImageProducer;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;

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