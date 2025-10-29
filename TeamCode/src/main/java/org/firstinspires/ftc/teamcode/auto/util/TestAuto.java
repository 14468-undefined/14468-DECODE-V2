package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="Test near")
public class TestAuto extends SampleAuto {
    private BaseRobot robot;



    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-48.5, 48.8, Math.toRadians(135)));


        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                //go to pile
                .strafeToSplineHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                //pick up
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))

                //return
                .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());
        robot.drive.updatePoseEstimate();



        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())


                .strafeToSplineHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 52.5))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(12, 50))
                .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))



                .build());
        robot.drive.updatePoseEstimate();


        //wait til shoot 3 is done to move

    }

    @Override
    public void onStop() {

    }
}