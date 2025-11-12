package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="BlueFarAutoMeet2")
public class BlueFarAutoMeet2 extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;


    int shooterRPM = 2800;
    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(61, -18, Math.toRadians(180)));
        shoot3 = new ShootCommand(robot, 3, 3);//3 artifacts far shot range
        robot.shooter.setTargetRPM(shooterRPM);
        //set pos of hood and transfer servo

    }

    @Override
    public void onStart() {


        while (opModeIsActive()) {
            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(204))//go to shoot pose
                    .build());

            AutoUtil.delay(2);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.5);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.5);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });

            AutoUtil.delay(.5);

            Actions.runBlocking((t) -> {
                robot.stopAll();
                return false;
            });
            // ====================== Intake 1st Pile ====================== \\


            //INTAKE PILE 1_________________________________________\\
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())


                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(4.8, (t) -> {
                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(4.85, (t) -> {
                        robot.transfer.spinReverse();
                        return false;
                    })
                    .afterTime(4.91, (t) -> {
                        robot.transfer.stop();
                        return false;
                    })
                    .afterTime(5, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })

                    .strafeToSplineHeading(new Vector2d(40, -29), Math.toRadians(270))//go to motif 1
                    .strafeToConstantHeading(new Vector2d(40, -66))//intake


                    .strafeToLinearHeading(new Vector2d(59, -10), Math.toRadians(199))//go to shoot pose

                    .build());


            AutoUtil.delay(2);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.45);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.6);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });

            AutoUtil.delay(.6);

            Actions.runBlocking((t) -> {
                robot.stopAll();
                return false;
            });
            // ====


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                            .strafeToConstantHeading(new Vector2d(59, -18))
                            .build());
            /*
            //INTAKE 2nd PILE IN CORNER
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(4.5, (t) -> {
                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(4.55, (t) -> {
                        robot.transfer.spinReverse();
                        return false;
                    })
                    .afterTime(4.61, (t) -> {
                        robot.transfer.stop();
                        return false;
                    })

                    .afterTime(4.7, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })
                    //get HP zone balls
                    .strafeToSplineHeading(new Vector2d(45, -75), Math.toRadians(0))//line up for HP zone balls
                    .strafeToSplineHeading(new Vector2d(61, -75), Math.toRadians(0))//line up for HP zone balls
                    //.strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                    .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(207.5))//go to shoot pose

                    .build());
                /*
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


                 */

        }

    }

    @Override
    public void onStop () {

    }


}