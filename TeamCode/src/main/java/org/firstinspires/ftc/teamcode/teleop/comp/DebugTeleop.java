package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "DebugTeleop" , group = "TeleOp")
public class DebugTeleop extends SampleCommandTeleop {

    int zone = 2; //mid default
    int numShots = 3;//number of shots
    double driveSpeed = 1;

    boolean shooterOn = false;

    int shooterRPM = 580;




    @Override
    public void onInit() {



        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));





        robot.shooter.setTargetRPM(shooterRPM);




    }

    @Override
    public void onStart() {


        robot.shooter.setTargetRPM(shooterRPM);

        /*
        DRIVE - normal robot centric

        g2 - DPAD controls = speed
         */
        //robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));

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
        INTAKING:
        LeftBumper = Outtake
        RightBumper = Intake
         */
        /*g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(robot.intake::intakeReverse);
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(robot.intake::intake);

         */




        /*
        SHOOTING:
        g1 X - shoot 3;
        g1 B - cancel;

        DPAD UP = far zone
        DPAD Right = mid zone
        DPAD Down = close zone (ur never in close zone)
         */



        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(() -> {
            robot.shooter.spinUp();
        });

        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(() -> {
            robot.shooter.stop();
        });



        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {
            robot.shooter.spinSlowReverse();
        });
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(() -> {
            robot.shooter.stop();
        });



        //right trigger = intake forwards, left trigger = intake reverse
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.intake();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.intakeReverse();
        });

        g2.getGamepadButton(GamepadKeys.Button.X).whenInactive(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.stop();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenInactive(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.stop();
        });







        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            shooterRPM = Constants.shooterConstants.MID_SHOT_RPM;

        });
        g2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM = Constants.shooterConstants.FAR_ZONE_SHOT_RPM;
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM += 50;
        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM -= 50;
        });



    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop
        robot.shooter.printTelemetry(pen);
        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
