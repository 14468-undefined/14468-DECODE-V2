package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity botMotif1 = new DefaultBotBuilder(meepMeep)
                .setDimensions(9, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity botMotif2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(9,18)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity botMotif3 = new DefaultBotBuilder(meepMeep)
                .setDimensions(9,18)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity botMotifAll = new DefaultBotBuilder(meepMeep)
                .setDimensions(9,18)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        botMotifAll.runAction(botMotifAll.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))
                // ==============Motif 1============== \\
                .strafeToLinearHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))

                // ==============Gate Open============== \\
                .strafeToLinearHeading(new Vector2d(1.4, 57.5),Math.toRadians(180))

                // ==============return============== \\
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))


                // ==============Motif 2============== \\
                .strafeToLinearHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 52.5))//intake

                // ==============return============== \\
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))

                // ==============Motif 3============== \\
                .strafeToLinearHeading(new Vector2d(35.8, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(35.8, 52.5))

                // ==============return============== \\
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))

                .build());


        botMotif3.runAction(botMotif3.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))
                .strafeToConstantHeading(new Vector2d(-11, 22))
                .strafeToLinearHeading(new Vector2d(35.8, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(35.8, 52.5))
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());

        botMotif2.runAction(botMotif2.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))
                        .strafeToConstantHeading(new Vector2d(-11, 22))
                .strafeToLinearHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 52.5))//intake
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());

        botMotif1.runAction(botMotif1.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))
                .strafeToLinearHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))
                .strafeToLinearHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());

        //.strafeToLinearHeading(new Vector2d(-48.5,48.8),135)
        //.splineToConstantHeading(new Vector2d(-11, 52.5), 90)
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botMotifAll)
                //.addEntity(botMotif1)
                //.addEntity(botMotif2)
                //.addEntity(botMotif3)
                .start();
    }
}