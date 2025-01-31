package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(30, 30, 8.107927703857422, Math.toRadians(60), 16.5)
                .setColorScheme(new ColorSchemeRedDark())
                /*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-15.23, -63.46, Math.toRadians(90.00)))
                                .lineToConstantHeading(new Vector2d(-2.97, -33.37))
                                .waitSeconds(1.5)
                                .lineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(65)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-57, -51, Math.toRadians(90.00)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-56.23, -55.09, Math.toRadians(125.00)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-55.54, -11.43, Math.toRadians(90)))
                                // Turn here
                                .lineToConstantHeading(new Vector2d(-25.00, -10.51))
                                .build()*/
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15.46, -64, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(4.62, -34.38))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(50.08, -51.92, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(59.54, -51.64))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-33))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(31, -57, Math.toRadians(-45)))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(4.62, -34.38, Math.toRadians(90)))
                                .build()
                );

        File background = new File("C:\\Users\\tartinger2025\\Downloads\\field-2024-juice-dark(2).png");
        Image image = ImageIO.read(background);

        meepMeep.setBackground(image)
                .setDarkMode(true)
                .addEntity(bot)
                .start();

    }
}