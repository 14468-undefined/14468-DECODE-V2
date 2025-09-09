package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;


@Autonomous
public final class TestAuto extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        double startY = -62.7;
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, -62.7, Math.toRadians(0)));

        robot.drive.resetHeading();
        waitForStart();



        //example action

        //movements here
        robot.update();

        Action MoveWithConstantHeadingExample = robot.drive.actionBuilder(robot.drive.localizer.getPose())//in past rr versions this was robot.drive.pose but changed with new API
                .splineToConstantHeading(new Vector2d(0, 0), 270)
                .build();
        Actions.runBlocking(MoveWithConstantHeadingExample);








    }
}