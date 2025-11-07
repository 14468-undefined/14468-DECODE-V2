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


    int shooterRPM = 3300;
    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(61, 18, Math.toRadians(180)));
        shoot3 = new ShootCommand(robot, 3, 3);//3 artifacts far shot range
        robot.shooter.setTargetRPM(shooterRPM);
        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {



            robot.shooter.spin();
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(153))//go to shoot pose
                    .build());

            robot.delay(2);
            robot.intake.intake();
            robot.delay(4);


            robot.intake.stop();
            robot.shooter.eStop();
            // ====================== Intake 1st Pile ====================== \\
            boolean[] intakeStarted = {false};
            boolean[] intakeStopped = {false};
            boolean[] shooterReverseStart = {false};
            boolean[] shooterReverseStop = {false};

            //INTAKE PILE 1_________________________________________\\
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())


                    .afterTime(0, t -> {
                        if (!intakeStarted[0]) {
                            robot.intake.intake();
                            intakeStarted[0] = true;
                        }
                        return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                    })

                    .afterTime(4.5, t -> {//was 3.7
                        if (!intakeStopped[0]) {
                            robot.intake.stop();
                            intakeStopped[0] = true;
                        }
                        return !intakeStopped[0]; // disables marker after stop fires
                    })

                    .afterTime(2, t -> {
                        if (!shooterReverseStart[0]) {
                            robot.shooter.setTargetRPM(-1000);
                            robot.shooter.spin();
                            shooterReverseStart[0] = true;
                        }
                        return !shooterReverseStart[0]; // disables marker after stop fires
                    })

                    .afterTime(5, t -> {
                        if (!shooterReverseStop[0]) {
                            robot.shooter.eStop();
                            robot.shooter.setTargetRPM(shooterRPM);
                            shooterReverseStop[0] = true;
                        }
                        return !shooterReverseStop[0]; // disables marker after stop fires
                    })

                    .strafeToSplineHeading(new Vector2d(24, 29), Math.toRadians(90))//go to motif 1
                    .strafeToConstantHeading(new Vector2d(24, 64))//intake


                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(160))//go to shoot pose

                    .build());

            robot.shooter.spin();
            robot.delay(3);
            robot.intake.intake();
            robot.delay(4);
            robot.intake.stop();
            robot.shooter.eStop();

            //INTAKE 2nd PILE IN CORNER
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .afterTime(0, t -> {
                        if (!intakeStarted[0]) {
                            robot.intake.intake();
                            intakeStarted[0] = true;
                        }
                        return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                    })

                    .afterTime(4.5, t -> {//was 3.7
                        if (!intakeStopped[0]) {
                            robot.intake.stop();
                            intakeStopped[0] = true;
                        }
                        return !intakeStopped[0]; // disables marker after stop fires
                    })

                    .afterTime(2, t -> {
                        if (!shooterReverseStart[0]) {
                            robot.shooter.setTargetRPM(-1000);
                            robot.shooter.spin();
                            shooterReverseStart[0] = true;
                        }
                        return !shooterReverseStart[0]; // disables marker after stop fires
                    })

                    .afterTime(5, t -> {
                        if (!shooterReverseStop[0]) {
                            robot.shooter.eStop();
                            robot.shooter.setTargetRPM(shooterRPM);
                            shooterReverseStop[0] = true;
                        }
                        return !shooterReverseStop[0]; // disables marker after stop fires
                    })
                    //get HP zone balls
                    .strafeToSplineHeading(new Vector2d(61, 65), Math.toRadians(0))//line up for HP zone balls
                    //.strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose

                    .build());

            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .afterTime(0, t -> {
                        if (!intakeStarted[0]) {
                            robot.intake.intake();
                            intakeStarted[0] = true;
                        }
                        return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                    })

                    .afterTime(4.5, t -> {//was 3.7
                        if (!intakeStopped[0]) {
                            robot.intake.stop();
                            intakeStopped[0] = true;
                        }
                        return !intakeStopped[0]; // disables marker after stop fires
                    })

                    .afterTime(2, t -> {
                        if (!shooterReverseStart[0]) {
                            robot.shooter.setTargetRPM(-1000);
                            robot.shooter.spin();
                            shooterReverseStart[0] = true;
                        }
                        return !shooterReverseStart[0]; // disables marker after stop fires
                    })

                    .afterTime(5, t -> {
                        if (!shooterReverseStop[0]) {
                            robot.shooter.eStop();
                            robot.shooter.setTargetRPM(shooterRPM);
                            shooterReverseStop[0] = true;
                        }
                        return !shooterReverseStop[0]; // disables marker after stop fires
                    })
                    .strafeToSplineHeading(new Vector2d(12, 29), Math.toRadians(90))//go to motif 2
                    .strafeToConstantHeading(new Vector2d(12, 61))//intake


                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(150))//go to shoot pose
                    .build());



    }

        @Override
        public void onStop () {

        }


    }