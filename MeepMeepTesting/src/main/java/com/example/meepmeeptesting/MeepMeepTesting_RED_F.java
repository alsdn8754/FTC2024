package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_RED_F {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(210.816), Math.toRadians(60), 15.24)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -65, 0))
                                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(0)))
                                .turn(Math.toRadians(20))
                                .lineToLinearHeading(new Pose2d(-39, -11.5, Math.toRadians(0)))
                                .splineTo(new Vector2d(31.5, -11.5), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(43.5, -42, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(43.5, -13, Math.toRadians(270)))
                                .strafeLeft(17)

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}