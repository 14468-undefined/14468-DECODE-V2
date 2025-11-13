package com.example.meepmeep;

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


        RoadRunnerBotEntity botTest = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                //.setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .setConstraints(200, 200, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)

                .build();



        botTest.runAction(botTest.getDrive().actionBuilder(new Pose2d(startX, startY, Math.toRadians(startHeading)))

                //pathing here
                        .splineTo(new Vector2d(50,50), Math.toRadians(270))
                .splineTo(new Vector2d(50,-50), Math.toRadians(90))
                .splineTo(new Vector2d(-50,-50), Math.toRadians(90))
                .splineTo(new Vector2d(-50,40), Math.toRadians(270))
                .splineTo(new Vector2d(50,-50), Math.toRadians(180))


                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botTest)

                .start();
    }
}