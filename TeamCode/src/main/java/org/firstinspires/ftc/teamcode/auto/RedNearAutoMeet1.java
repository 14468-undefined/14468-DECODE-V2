package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));
        //shoot3 = new ShootCommand(robot, 3, 2);//3 artifacts mid range shot
        robot.shooter.setTargetRPM(1000);
        //set pos of hood and transfer servo


    }

    @Override
    public void onStart() {

        robot.shooter.spinUpChooseRPM(585);
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToSplineHeading(new Vector2d(-25,24),Math.toRadians(130))//go to shooting pose
                .build());
        //shoot3.schedule();

        /*while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }

         */
        //wait til 3 are shot to move

        robot.intake.intake();
        robot.delay(4);
        robot.intake.stop();
        robot.shooter.stop();
        // ====================== Intake 1st Pile ====================== \\jacob cant code an auto ts is why we dont make worlds now im gonna type out the alphabet abcdefghijk,lmnop
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(0, t -> {robot.intake.intake();  return true;})
                //stop intake after 4 sec
                .afterTime(3, t -> {robot.intake.stop();  return  true;})

                //.afterTime(5, t-> {robot.shooter.spinUpChooseRPM(600); return true;})

                //go to pile
                .strafeToSplineHeading(new Vector2d(-16.5, 25.4), Math.toRadians(90))
                //pick up
                .strafeToConstantHeading(new Vector2d(-19, 58))

                /*gate dump
                .strafeToConstantHeading(new Vector2d(-11.2, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump
                 */

                //return
                .strafeToSplineHeading(new Vector2d(-25,24),Math.toRadians(130))
                .build());


        //robot.intake.intakeReverse();
        //robot.delay(.1);
        //robot.intake.stop();
        //shoot 3 artifacts
        robot.shooter.spinUpChooseRPM(580);
        robot.delay(1);
        robot.intake.intake();
        robot.delay(4);
        robot.intake.stop();
        robot.shooter.stop();
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