package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "MeetTwoTeleop" , group = "AA - COMP")
public class MeetTwoTeleop extends SampleCommandTeleop {


    double driveSpeed = 1;

    boolean shooterOn = false;

    int shooterRPM = 2135;




    @Override
    public void onInit() {



        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY() * driveSpeed, -g1.getLeftX() * driveSpeed), -g1.getRightX() * driveSpeed)), robot.drive));





        robot.shooter.setTargetRPM(shooterRPM);

        double shooterRealRPM = robot.shooter.getShooterVelocity();


    }

    @Override
    public void onStart() {





        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {

            if(robot.shooter.isAtTargetSpeed()){
                robot.LED.setColor(LEDSubsystem.LEDColor.GREEN);
            }
            else{
                robot.LED.setColor(LEDSubsystem.LEDColor.RED);
            }
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            driveSpeed = 1;
        });
        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            driveSpeed = .5;
        });
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            driveSpeed = .2;
        });





        /*
        SHOOTING:
        g1 X - shoot 3;
        g1 B - cancel;

        DPAD UP = far zone
        DPAD Right = mid zone
        DPAD Down = close zone (ur never in close zone)
         */



        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(() -> {
            robot.shooter.setTargetRPM(shooterRPM);
            robot.shooter.spin();
        });

        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(() -> {
            robot.shooter.eStop();
        });



        /*
        shooter reverse

        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {
            robot.shooter.setTargetRPM(-1000);
            robot.shooter.spin();
        });
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(() -> {
            robot.shooter.eStop();
        });


         */


        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {
            robot.transfer.spin();
            robot.intake.intake();
        });
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(() -> {
                    robot.intake.stop();
                    robot.transfer.stop();
        });

        //right trigger = intake forwards, left trigger = intake reverse
        //g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));


        new Trigger(() -> gamepad1.right_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));
        new Trigger(() -> gamepad1.left_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.intake();
            //robot.transfer.spinReverse();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.intakeReverse();
            robot.transfer.spinReverse();
        });

        g2.getGamepadButton(GamepadKeys.Button.X).whenReleased(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.stop();
            robot.transfer.stop();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.stop();
            robot.transfer.stop();
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(() -> {
            //zone = 3;
            //numShots = 3;

            robot.transfer.stop();
        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;

            robot.transfer.spinReverse();
        });






        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            shooterRPM = 2135;
            robot.shooter.setTargetRPM(shooterRPM);

        });
        g2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM = 2435;
            robot.shooter.setTargetRPM(shooterRPM);
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM += 50;
            robot.shooter.setTargetRPM(shooterRPM);
        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM -= 50;
            robot.shooter.setTargetRPM(shooterRPM);
        });




    }

    @Override
    public void onLoop() {


        // Print intake telemetry every loopq
        pen.addData("Shooter RPM: ", robot.shooter.getShooterVelocity());
        pen.addData("Set RPM: ", robot.shooter.getTargetRPM());

        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
