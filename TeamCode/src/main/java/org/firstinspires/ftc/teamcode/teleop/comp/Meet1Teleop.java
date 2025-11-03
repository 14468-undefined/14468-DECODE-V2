package org.firstinspires.ftc.teamcode.teleop.comp;

import static com.arcrobotics.ftclib.kotlin.extensions.gamepad.GamepadExExtKt.whenActive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;

import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.command.SpinUpShooterCommand;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;


@TeleOp
public class Meet1Teleop extends SampleCommandTeleop {

    int zone = 2; //mid default
    int numShots = 3;//number of shots
    double driveSpeed = 1;

    boolean shooterOn = false;

    Command Shoot = new ShootCommand(robot, zone, numShots);
    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));

        waitForStart();


    }

    @Override
    public void onStart() {



        /*
        DRIVE - normal robot centric

        g2 - DPAD controls = speed
         */
        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));

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
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(robot.intake::intakeReverse);
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(robot.intake::intake);




        /*
        SHOOTING:
        g1 X - shoot 3;
        g1 B - cancel;

        DPAD UP = far zone
        DPAD Right = mid zone
        DPAD Down = close zone (ur never in close zone)
         */
        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
                    ShootCommand shootCommand = new ShootCommand(robot, numShots, zone);
                    shootCommand.schedule();
        });
        g1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
                    // Cancel command
                    CommandScheduler.getInstance().cancel(robot.shooter.getCurrentCommand());
        });


        if(shooterOn){
            robot.shooter.spinUp();
        }
        if(!shooterOn){
            robot.shooter.stop();
        }
        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(()-> {shooterOn = true;});
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(()-> {shooterOn = false;});
        g2.getGamepadButton(GamepadKeys.Button.Y).whenActive(new ShootCommand(robot, numShots, zone));



        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            zone = 2;
            numShots = 3;

        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            zone = 3;
            numShots = 3;
        });

        //spin reverse at power of .2 - really -.2 but reversed in subsystem
        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            robot.shooter.spinSlowReverse();
        });

    }

    @Override
    public void onLoop() {
        telemetry.addLine(g1.getRightX()+"");
        telemetry.addLine(g1.getRightY()+"");
        telemetry.addLine(g1.getLeftX()+"");
        telemetry.addLine(g1.getLeftY()+"");


    }

    @Override
    public void onStop() {

    }
}