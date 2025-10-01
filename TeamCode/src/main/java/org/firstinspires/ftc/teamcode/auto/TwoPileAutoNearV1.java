package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.command.Shoot3Command;
import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="TwoPileAutoNearV1")
public class TwoPileAutoNearV1 extends SampleAuto {
    private BaseRobot robot;
    private Shoot3Command shoot3;
    private IntakeCommand intake;


    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-48.5, 48.8, Math.toRadians(135)));
        shoot3 = new Shoot3Command(robot, 3);
        intake = new IntakeCommand(robot);
        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {

        shoot3.schedule();
        while (!shoot3.isFinished() && opModeIsActive()) {
            CommandScheduler.getInstance().run(); //run scheduler
            idle(); //
        }
        //wait til shoot 3 is done to move


        // ====================== Intake 1st Pile ====================== \\
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                // Start intake after 4 seconds
                .afterTime(2.3, t -> {intake.schedule(); return true;})
                     //stop intake after 7 sec
                .afterTime(3.5, t -> {robot.intake.stop();  return true;})


                //go to pile
                .strafeToSplineHeading(new Vector2d(-11.2, 22.4), Math.toRadians(90))
                //pick up
                .strafeToConstantHeading(new Vector2d(-11.2, 52.5))

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

                // Start intake after 4 seconds
                .afterTime(2.8, t -> {robot.intake.intake(); return true;})
                .afterTime(2.8, t -> {robot.transfer.spinReverse(); return true;})
                //stop intake after 7 sec
                .afterTime(4, t -> {robot.intake.stop();  return true;})
                .afterTime(4, t -> {robot.transfer.stop(); return true;})

                //drive to balls
                .strafeToSplineHeading(new Vector2d(12, 22.4), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(12, 52.5))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(12, 50))
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

    }

    @Override
    public void onStop() {

    }
}