package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();


        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();


        blueBot.runAction(blueBot.getDrive().actionBuilder(new Pose2d(56, -9, Math.toRadians(215)))



                //.strafeToLinearHeading(new Vector2d(35.8, 22.4), Math.toRadians(90))
                //.strafeToConstantHeading(new Vector2d(35.8, 52.5))
                .strafeToLinearHeading(new Vector2d(38.5, -35), Math.toRadians(255))
                .strafeToLinearHeading(new Vector2d(35.8, -52.5), Math.toRadians(270))

                .strafeToLinearHeading(new Vector2d(56, -9), Math.toRadians(225))
                .build());

        redBot.runAction(redBot.getDrive().actionBuilder(new Pose2d(61, 18, Math.toRadians(180)))


                //SMOOTHER pathing but less reliable intake prob
                //.strafeToLinearHeading(new Vector2d(38.5, 35), Math.toRadians(105))
                //.strafeToLinearHeading(new Vector2d(35.8, 52.5), Math.toRadians(90))



                .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif 1
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake

                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose

                //get HP zone balls
                .strafeToSplineHeading(new Vector2d(44, 65), Math.toRadians(0))//line up for HP zone balls
                .strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose


                .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif 2
                .strafeToConstantHeading(new Vector2d(12, 61))//intake

                //.strafeToSplineHeading(new Vector2d(50, 57), Math.toRadians(75))//alternate path to corner
                //.strafeToSplineHeading(new Vector2d(60, 58), Math.toRadians(85))//alternate path to corner

                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .addEntity(blueBot)
                //.addEntity(botMotif1)
                //.addEntity(botMotif2)
                //.addEntity(botMotif3)
                .start();
    }
}