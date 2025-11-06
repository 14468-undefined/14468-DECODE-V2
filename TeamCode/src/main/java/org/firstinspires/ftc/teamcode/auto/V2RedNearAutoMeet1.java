package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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

@Autonomous(name="V2RedNearAutoMeet1")
public class V2RedNearAutoMeet1 extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;

    private int shooterRPMClose = 575;

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));
        //shoot3 = new ShootCommand(robot, 3, 2);//3 artifacts mid range shot
        robot.shooter.setTargetRPM(570);
        //set pos of hood and transfer servo
        robot.intake.setIntakePower(1);

    }

    @Override
    public void onStart() {

        robot.shooter.spinUpChooseRPM(shooterRPMClose+30);
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
        robot.delay(2.9);
        //robot.shooter.spinUpChooseRPM(shooterRPMClose+55);
        //robot.delay(1);
        robot.intake.stop();
        robot.shooter.stop();
        // ====================== Intake 1st Pile ====================== \\
        boolean[] intakeStarted = {false};
        boolean[] intakeStopped = {false};
        boolean[] shooterReverseStart = {false};
        boolean[] shooterReverseStop = {false};

        robot.intake.setIntakePower(1);
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(0, t -> {
                    if (!intakeStarted[0]) {
                        robot.intake.intake();
                        intakeStarted[0] = true;
                    }
                    return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                })

                .afterTime(2.8, t -> {//was 3.7
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

                .afterTime(4.5, t -> {
                    if (!shooterReverseStop[0]) {
                        robot.shooter.stop();
                        shooterReverseStop[0] = true;
                    }
                    return !shooterReverseStop[0]; // disables marker after stop fires
                })


                //NEW - SLOWER MOVEMENT

                //.strafeToConstantHeading(new Vector2d(54, -60), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100 , 100))
                .strafeToSplineHeading(new Vector2d(-16.5, 25.4), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-19, 63.5), new TranslationalVelConstraint(30))


                //.strafeToSplineHeading(new Vector2d(-16.5, 25.4), Math.toRadians(90))

                //.strafeToConstantHeading(new Vector2d(-19, 63.5))//get last 2
                .strafeToConstantHeading(new Vector2d(-24, 57))
                .strafeToSplineHeading(new Vector2d(-29, 24), Math.toRadians(138))
                .build());

        robot.shooter.spinUpChooseRPM(shooterRPMClose+20);
        robot.delay(2);
        robot.intake.setIntakePower(1);
        robot.intake.intake();
        robot.delay(3);
        robot.intake.stop();
        robot.shooter.stop();
        //wait til shoot 3 is done to move



        intakeStarted[0] = false;
        intakeStopped[0] = false;
        shooterReverseStart[0] = false;
        shooterReverseStop[0] = false;

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(1.6, t -> {
                    if (!intakeStarted[0]) {
                        robot.intake.intake();
                        intakeStarted[0] = true;
                    }
                    return !intakeStarted[0]; // becomes false after first trigger
                })

                .afterTime(4.4 , t -> {
                    if (!intakeStopped[0]) {
                        robot.intake.stop();
                        intakeStopped[0] = true;
                    }
                    return !intakeStopped[0];
                })

                .afterTime(2, t -> {
                    if (!shooterReverseStart[0]) {
                        robot.shooter.spinSlowReverse();
                        shooterReverseStart[0] = true;
                    }
                    return !shooterReverseStart[0]; // disables marker after stop fires
                })

                .afterTime(4.6, t -> {
                    if (!shooterReverseStop[0]) {
                        robot.shooter.stop();
                        shooterReverseStop[0] = true;
                    }
                    return !shooterReverseStop[0]; // disables marker after stop fires
                })


                //MOTIF 2
                .strafeToSplineHeading(new Vector2d(8, 22), Math.toRadians(90))//go to motif
                .strafeToConstantHeading(new Vector2d(8, 66))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(9, 50))//back up
                .strafeToSplineHeading(new Vector2d(-27,24),Math.toRadians(140))//shooting pose



                .build());

        robot.shooter.spinUpChooseRPM(shooterRPMClose+30);
        robot.delay(2);
        robot.intake.intake();
        robot.delay(2);
        robot.shooter.spinUpChooseRPM(shooterRPMClose+30);
        robot.delay(2);
        robot.intake.stop();
        robot.shooter.stop();




    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}