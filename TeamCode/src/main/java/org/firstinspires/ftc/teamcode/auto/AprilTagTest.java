package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystem.Actions.DriveToAprilTagAction;

@Autonomous(name = "AprilTagTest")
public final class AprilTagTest extends LinearOpMode {
    BaseRobot robot;
    AprilTagVision aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // Start pose: X = -39, Y = -62.7, Heading = 0
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, -62.7, Math.toRadians(0)));
        aprilTag = new AprilTagVision(hardwareMap);

        robot.drive.resetHeading();

        waitForStart();

        if (isStopRequested()) return;


        Action moveExample = robot.drive.actionBuilder(robot.drive.localizer.getPose())
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(270))
                .build();
        Actions.runBlocking(moveExample);

        // Drive to *any* visible AprilTag
        Action driveToAnyTag = new DriveToAprilTagAction(robot.drive, aprilTag);
        Actions.runBlocking(driveToAnyTag);

        // OR: Drive to a specific tag ID, e.g. Tag 3
        // Action driveToTag3 = new DriveToAprilTagAction(robot.drive, aprilTag, 3);
        // Actions.runBlocking(driveToTag3);

        aprilTag.close();
    }
}