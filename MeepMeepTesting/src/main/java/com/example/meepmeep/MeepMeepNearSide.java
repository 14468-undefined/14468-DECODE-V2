package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepNearSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);






        RoadRunnerBotEntity redBot3Piles = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();

        RoadRunnerBotEntity redBot2Piles = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();

        //==============================================BLUE==============================================\\
        RoadRunnerBotEntity blueBot3Piles = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();

        RoadRunnerBotEntity blueBot2Piles = new DefaultBotBuilder(meepMeep)
                .setDimensions(Constants.ROBOT_WIDTH, Constants.ROBOT_HEIGHT)
                .setConstraints(Constants.MAX_VEL, Constants.MAX_ACCEL, Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACCEL, Constants.TRACK_WIDTH)
                .build();




        redBot2Piles.runAction(redBot2Piles.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, Math.toRadians(135)))
                // ==============Motif 1============== \\
                .strafeToSplineHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))



                // ==============return============== \\
                .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))


                // ==============Motif 2============== \\
                .strafeToSplineHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 52.5))//intake

                // ==============return============== \\
                        .strafeToConstantHeading(new Vector2d(12, 50))
                .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))



                .build());




                redBot3Piles.runAction(redBot3Piles.getDrive().actionBuilder(new Pose2d(-48.5, 48.8, 135))

                        // ==============Motif 1============== \\
                        .strafeToSplineHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-11.2, 52.5))

                        // ==============Gate Open============== \\
                        .strafeToSplineHeading(new Vector2d(1.4, 57.5),Math.toRadians(180))

                        // ==============return============== \\
                        .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))


                        // ==============Motif 2============== \\
                        .strafeToSplineHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                        .strafeToConstantHeading(new Vector2d(12, 52.5))//intake

                        // ==============return============== \\
                        .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))

                        // ==============Motif 3============== \\
                        .strafeToSplineHeading(new Vector2d(35.8, 22.4), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(35.8, 52.5))

                        // ==============return============== \\
                        .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))

                        .build());



        blueBot2Piles.runAction(blueBot2Piles.getDrive().actionBuilder(new Pose2d(-48.5, -48.8, Math.toRadians(225)))
                // ==============Motif 1============== \\
                .strafeToSplineHeading(new Vector2d(-11.2, -22.4), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-11.2, -52.5))



                // ==============return============== \\
                .strafeToSplineHeading(new Vector2d(-48.5,-48.8),Math.toRadians(225))


                // ==============Motif 2============== \\
                .strafeToSplineHeading(new Vector2d(12, -22.4), Math.toRadians(270))//go to motif
                .strafeToConstantHeading(new Vector2d(12, -52.5))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(12, -50))
                .strafeToSplineHeading(new Vector2d(-48.5,-48.8),Math.toRadians(225))



                .build());




        //.strafeToLinearHeading(new Vector2d(-48.5,48.8),135)
        //.splineToConstantHeading(new Vector2d(-11, 52.5), 90)
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBot2Piles)
                .addEntity(redBot2Piles)
                //.addEntity(botMotif1)
                //.addEntity(botMotif2)
                //.addEntity(botMotif3)
                .start();
    }
}