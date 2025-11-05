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

    private int shooterRPMClose = 585;

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));
        //shoot3 = new ShootCommand(robot, 3, 2);//3 artifacts mid range shot
        robot.shooter.setTargetRPM(1000);
        //set pos of hood and transfer servo


    }

    @Override
    public void onStart() {

        robot.shooter.spinUpChooseRPM(shooterRPMClose);
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
        // ====================== Intake 1st Pile ====================== \\
        boolean[] intakeStarted = {false};
        boolean[] intakeStopped = {false};
        boolean[] shooterReverseStart = {false};
        boolean[] shooterReverseStop = {false};

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(0, t -> {
                    if (!intakeStarted[0]) {
                        robot.intake.intake();
                        intakeStarted[0] = true;
                    }
                    return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                })

                .afterTime(3.5, t -> {
                    if (!intakeStopped[0]) {
                        robot.intake.stop();
                        intakeStopped[0] = true;
                    }
                    return !intakeStopped[0]; // disables marker after stop fires
                })

                .afterTime(2, t -> {
                    if (!shooterReverseStart[0]) {
                        robot.shooter.spinSlowReverse();
                        shooterReverseStart[0] = true;
                    }
                    return !shooterReverseStart[0]; // disables marker after stop fires
                })

                .afterTime(3, t -> {
                    if (!shooterReverseStop[0]) {
                        robot.shooter.stop();
                        shooterReverseStop[0] = true;
                    }
                    return !shooterReverseStop[0]; // disables marker after stop fires
                })


                .strafeToSplineHeading(new Vector2d(-16.5, 25.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-19, 60))
                .strafeToSplineHeading(new Vector2d(-28, 24), Math.toRadians(138))
                .build());

        robot.shooter.spinUpChooseRPM(shooterRPMClose);
        robot.delay(2);
        robot.intake.intake();
        robot.delay(4);
        //robot.intake.stop();
        //robot.shooter.stop();
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
                .strafeToSplineHeading(new Vector2d(-24,24),Math.toRadians(140))//shooting pose



                .build());

        robot.shooter.spinUpChooseRPM(shooterRPMClose);
        robot.delay(1);
        robot.intake.intake();
        robot.delay(4);
        robot.intake.stop();
        robot.shooter.stop();




    }

    @Override
    public void onStop() {

    }
}