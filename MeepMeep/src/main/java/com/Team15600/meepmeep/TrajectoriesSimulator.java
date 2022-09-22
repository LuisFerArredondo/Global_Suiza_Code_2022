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
import javax.imageio.ImageReader;

public class TrajectoriesSimulator {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(TestTrajectory.traj(meepMeep))
                //.addEntity(mySecondBot)
                .start();
    }
}