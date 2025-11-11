package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="RedNear9Artifacts")
public class RedNear9Artifacts extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;

    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit(){
        
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));
        //shoot3 = new ShootCommand(robot, 3, 2);//3 artifacts mid range shot
        robot.shooter.setTargetRPM(shooterRPMClose);
        //set pos of hood and transfer servo
        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


    }

    @Override
    public void onStart() {

        while(opModeIsActive()) {

            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToSplineHeading(new Vector2d(-27, 24), Math.toRadians(133))//go to shooting pose
                    .build());


            Actions.runBlocking((t) -> {
                robot.intake.intake();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                return false;
            });

            AutoUtil.delay(2.9);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.intake.setIntakePower(1);
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.shooter.eStop();
                return false;
            });

            // ====================== Intake 1st Pile ====================== \\

            robot.intake.setIntakePower(1);
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                    .afterTime(0, t -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(3, t -> {//was 3.7

                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(3.1, (t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.19, (t) -> {
                        robot.transfer.stop();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.3, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })


                    .strafeToSplineHeading(new Vector2d(-17, 25.4), Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(-19, 64.3), new TranslationalVelConstraint(30))


                    .strafeToConstantHeading(new Vector2d(-24, 57))
                    .strafeToSplineHeading(new Vector2d(-30, 24), Math.toRadians(145))
                    .build());

            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });
            //AutoUtil.delay(2);
            Actions.runBlocking((t) -> {
                robot.intake.setIntakePower(1);
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.intake.intake();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                return false;
            });
            AutoUtil.delay(3);
            Actions.runBlocking((t) -> {
                robot.stopAll();
                return false;
            });
            //wait til shoot 3 is done to move


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                    .afterTime(1.6, t -> {

                        robot.intake.intake();
                        return false; // becomes false after first trigger
                    })

                    .afterTime(3.7, t -> {

                        robot.intake.stop();
                        return false;
                    })


                    .afterTime(3.8, (t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.89, (t) -> {
                        robot.transfer.stop();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(4, (t) -> {
                        robot.shooter.spin();
                        //robot.transfer.stop();
                        return false;
                    })


                    //MOTIF 2
                    .strafeToSplineHeading(new Vector2d(3, 22), Math.toRadians(90))//go to motif
                    .strafeToConstantHeading(new Vector2d(3, 69))//intake

                    // ==============return============== \\
                    .strafeToConstantHeading(new Vector2d(9, 50))//back up
                    .strafeToSplineHeading(new Vector2d(-36, 24), Math.toRadians(143))//shooting pose


                    .build());


            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });
            //AutoUtil.delay(2);
            Actions.runBlocking((t) -> {
                robot.intake.intake();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                return false;
            });
            AutoUtil.delay(2);
            Actions.runBlocking((t) -> {
                robot.shooter.eStop();
                return false;
            });
            AutoUtil.delay(2);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.shooter.eStop();
                return false;
            });
            break;
        }
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}