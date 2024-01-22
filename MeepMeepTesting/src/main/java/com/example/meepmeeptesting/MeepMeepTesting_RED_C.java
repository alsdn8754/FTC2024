package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_RED_C {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(210.816), Math.toRadians(60), 15.24)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.5, -65, 0)) //no

                                .lineToLinearHeading(new Pose2d(11.5, -41, Math.toRadians(130)))

                                .lineToLinearHeading(new Pose2d(43.5, -37, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(37.1, -5.1, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-59.8, -11.1, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(37.1, -10.5, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(43.5, -37, Math.toRadians(0)))

                                .lineToLinearHeading(new Pose2d(43.5, -60, Math.toRadians(270)))
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