package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(9, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();



        bot1.runAction(bot1.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))
                .strafeToLinearHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                //.addEntity(botMotif1)
                //.addEntity(botMotif2)
                //.addEntity(botMotif3)
                .start();
    }
}