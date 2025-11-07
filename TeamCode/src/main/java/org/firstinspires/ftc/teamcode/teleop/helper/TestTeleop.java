package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "TestTeleop" , group = "TeleOp")
public class TestTeleop extends SampleCommandTeleop {

    //BaseRobot robot;
    FtcDashboard dash;
    
    HardwareMap hwMap;

    int shooterRPM = 6000;


    @Override
    public void onInit() {



        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));




        //robot.drive.setDefaultCommand(robot.drive.getDriveFieldcentric(()->g1.getLeftY(),()->-g1.getLeftX(), ()->-g1.getRightX(), .75));

        robot.shooter.setTargetRPM(shooterRPM);




    }

    @Override
    public void onStart() {




        g1.getGamepadButton(GamepadKeys.Button.Y).whenActive(()->robot.drive.drive.resetHeadingRelative());

        //robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));



        //X = SPIN UP SHOOTER



        //RIGHT BUMPER = intake; LEFT BUMPER = Intake reverse;
        g1.getGamepadButton(GamepadKeys.Button.A).whileHeld(robot.intake::intake);
        g1.getGamepadButton(GamepadKeys.Button.A).whenReleased(robot.intake::stop);

        g1.getGamepadButton(GamepadKeys.Button.Y).whileHeld(robot.intake::intakeReverse);
        g1.getGamepadButton(GamepadKeys.Button.Y).whenReleased(robot.intake::stop);


        //SHOOTER
        //DPAD_UP = power + 100; DPAD_DOWN = power - 100;
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            shooterRPM += 100;

            robot.shooter.setTargetRPM(shooterRPM);
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            shooterRPM -= 100;

            robot.shooter.setTargetRPM(shooterRPM);
        });

        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spin);
        g1.getGamepadButton(GamepadKeys.Button.X).whenReleased(robot.shooter::eStop);


        //g1.getGamepadButton(GamepadKeys.Button.B).whileHeld(robot.shooter::spinSlowReverse);
        //g1.getGamepadButton(GamepadKeys.Button.B).whenReleased(robot.shooter::spinSlowReverse);


        pen.addLine("CONTROLS");
        pen.addLine();
        pen.addLine("Intake: A");
        pen.addLine("Shooter: X");
        pen.addLine("Power Up: DPAD_UP");
        pen.addLine("Power Down: DPAD_DOWN");



    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop

        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
