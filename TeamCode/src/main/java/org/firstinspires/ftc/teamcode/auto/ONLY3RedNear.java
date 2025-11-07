package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="ONLY3RedNear")
public class ONLY3RedNear extends SampleAuto {
    private BaseRobot robot;

    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);

        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


    }

    @Override
    public void onStart() {

        robot.shooter.spin();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToSplineHeading(new Vector2d(-27,24),Math.toRadians(133))//go to shooting pose
                .build());




        robot.intake.intake();
        robot.delay(2.9);

        robot.intake.stop();
        robot.intake.setIntakePower(1);
        robot.shooter.eStop();
        //PARK
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())



                .strafeToSplineHeading(new Vector2d(-55, 24), Math.toRadians(180))
                .build());


    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}