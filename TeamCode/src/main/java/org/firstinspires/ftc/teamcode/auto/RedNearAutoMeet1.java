package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="RedNearAutoMeet1")
public class RedNearAutoMeet1 extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;


    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-48.5, 48.8, Math.toRadians(135)));
        shoot3 = new ShootCommand(robot, 3, 2);//3 artifacts mid range shot
        robot.shooter.setTargetRPM(Constants.shooterConstants.MID_SHOT_RPM);
        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {

        robot.shooter.spinUp();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//go to shooting pose
                .build());
        shoot3.schedule();

        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }
        //wait til 3 are shot to move

        // ====================== Intake 1st Pile ====================== \\
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(2.3, t -> {robot.intake.intake();  return true;})
                //stop intake after 4 sec
                .afterTime(4, t -> {robot.intake.stop();  return true;})

                .afterTime(5, t-> {robot.shooter.spinUp(); return true;})

                //go to pile
                .strafeToSplineHeading(new Vector2d(-11.2, 25.4), Math.toRadians(90))
                //pick up
                .strafeToConstantHeading(new Vector2d(-11.2, 54.5))

                /*gate dump
                .strafeToConstantHeading(new Vector2d(-11.2, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump
                 */

                //return
                .strafeToSplineHeading(new Vector2d(-48.5,48.8),Math.toRadians(135))
                .build());
        robot.drive.updatePoseEstimate();


        //shoot 3 artifacts
        shoot3.schedule();
        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }
        //wait til shoot 3 is done to move


        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                // Start intake after 2.8 seconds
                .afterTime(1.6, t -> {robot.intake.intake(); return true;})

                //stop intake after 4 sec
                .afterTime(4, t -> {robot.intake.stop();  return true;})

                .afterTime(5, t-> {robot.shooter.spinUp(); return true;})

                //MOTIF 2
                .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 61))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(12, 50))//back up
                .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(135))//shooting pose



                .build());
        robot.drive.updatePoseEstimate();


        //shoot 3 artifacts
        shoot3.schedule();
        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }
        //wait til shoot 3 is done to move

    }

    @Override
    public void onStop() {

    }
}