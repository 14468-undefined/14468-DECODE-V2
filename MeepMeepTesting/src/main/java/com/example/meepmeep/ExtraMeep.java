package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class ExtraMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int startX = 0;
        int startY = 0;
        int startHeading = 0;


        RoadRunnerBotEntity undefinedBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                //.setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)

                .build();

        RoadRunnerBotEntity BOTbot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                        .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();



        undefinedBot.runAction(undefinedBot.getDrive().actionBuilder((new Pose2d(-61, 40, Math.toRadians(180))))

                        .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//go shoot
                                .waitSeconds(4)
                //go to pile
                .strafeToSplineHeading(new Vector2d(-11.2, 25.4), Math.toRadians(90))
                //pick up
                .strafeToConstantHeading(new Vector2d(-11.2, 54.5))

                //gate dump
                .strafeToConstantHeading(new Vector2d(-11.2, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump
                                .strafeToConstantHeading(new Vector2d(-17, 48))
                //return\
                .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//shoot

                        .waitSeconds(4)
                .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif 1
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake
                .strafeToConstantHeading(new Vector2d(35.8, 29))//intake
                .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//go shoot

                .build());

        BOTbot.runAction(BOTbot.getDrive().actionBuilder((new Pose2d(-41, 53, Math.toRadians(90))))

                .strafeToSplineHeading(new Vector2d(-17,17),Math.toRadians(135))//go shoot
                                .waitSeconds(4)

                .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 61))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(12, 20))
                .strafeToSplineHeading(new Vector2d(-17,17),Math.toRadians(135))//shoot
                        .waitSeconds(4)


                //.strafeToSplineHeading(new Vector2d(46.4, 61.7), Math.toRadians(0))
                //.strafeToConstantHeading(new Vector2d(58, 61.7))

                .strafeToSplineHeading(new Vector2d(60, 29), Math.toRadians(90))//go to corner
                .strafeToConstantHeading(new Vector2d(60, 59))
                        .waitSeconds(4)

               // .strafeToSplineHeading(new Vector2d(-17,17),Math.toRadians(135))//shoot

                .strafeToLinearHeading(new Vector2d(56, 14), Math.toRadians(154))//go to shoot pose
                        .waitSeconds(4)


                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(undefinedBot)
                .addEntity(BOTbot)

                .start();
    }
}