package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="Test near")
public class TestAuto extends SampleAuto {
    private BaseRobot robot;



    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-48.5, 48.8, Math.toRadians(135)));




    }

    @Override
    public void onStart() {

        Actions.runBlocking(robot.drive.actionBuilder(new Pose2d(-61, 40, Math.toRadians(180)))

                        .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//go shoot
                        //go to pile
                        .strafeToSplineHeading(new Vector2d(-11.2, 25.4), Math.toRadians(90))
                        //pick up
                        .strafeToConstantHeading(new Vector2d(-11.2, 54.5))

                        //gate dump
                        //.strafeToConstantHeading(new Vector2d(-11.2, 48))
                        //.strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump
                        //return
                        .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//shoot


                        //MOTIF 2
                        .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif
                        .strafeToConstantHeading(new Vector2d(12, 61))//intake

                        // ==============return============== \\
                        .strafeToConstantHeading(new Vector2d(12, 50))
                        .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//shoot

                        //motif 3
                        .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif
                        .strafeToConstantHeading(new Vector2d(35.8, 61))//intake

                        //return
                        .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//shoot

                        .build());


        //wait til shoot 3 is done to move

    }

    @Override
    public void onStop() {

    }
}