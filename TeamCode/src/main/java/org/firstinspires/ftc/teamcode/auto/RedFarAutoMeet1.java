package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name="RedFarAutoMeet1")
public class RedFarAutoMeet1 extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;


    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(35.8, 29, Math.toRadians(180)));
        shoot3 = new ShootCommand(robot, 3, 3);//3 artifacts far shot range
        robot.shooter.setTargetRPM(Constants.shooterConstants.MID_SHOT_RPM);
        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {

        robot.shooter.spinUp();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose
                .build());
        shoot3.schedule();

        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }
        //wait til 3 are shot to move


        //INTAKE PILE 1_________________________________________\\
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(1.5, t -> {robot.intake.intake();  return true;})
                //stop intake after 4 sec
                .afterTime(3.5, t -> {robot.intake.stop();  return true;})
                .afterTime(3, t -> {robot.shooter.spinSlowReverse(); return true;})

                .afterTime(5, t-> {robot.shooter.spinUp(); return true;})


                .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif 1
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake


                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose

                .build());

        shoot3.schedule();

        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }

        //INTAKE 2nd PILE IN CORNER
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(1.5, t -> {robot.intake.intake();  return true;})
                //stop intake after 4 sec
                .afterTime(3.5, t -> {robot.intake.stop();  return true;})
                .afterTime(3, t -> {robot.shooter.spinSlowReverse(); return true;})

                .afterTime(5, t-> {robot.shooter.spinUp(); return true;})
                //get HP zone balls
                .strafeToSplineHeading(new Vector2d(44, 65), Math.toRadians(0))//line up for HP zone balls
                .strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose

                .build());

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .afterTime(1.5, t -> {robot.intake.intake();  return true;})
                //stop intake after 4 sec
                .afterTime(3.5, t -> {robot.intake.stop();  return true;})
                .afterTime(3, t -> {robot.shooter.spinSlowReverse(); return true;})

                .afterTime(5, t-> {robot.shooter.spinUp(); return true;})
                .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif 2
                .strafeToConstantHeading(new Vector2d(12, 61))//intake


                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose
                .build());

    }

        @Override
        public void onStop () {

        }


    }