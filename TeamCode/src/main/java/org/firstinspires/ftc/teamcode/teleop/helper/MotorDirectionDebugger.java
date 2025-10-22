package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "MotorDirectionDebugger" , group = "TeleOp")
public class MotorDirectionDebugger extends SampleCommandTeleop {

    //BaseRobot robot;
    FtcDashboard dash;
    
    HardwareMap hwMap;

    int shooterRPM = 6000;


    @Override
    public void onInit() {









    }

    @Override
    public void onStart() {



        //robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));


        robot.shooter.setTargetRPM(shooterRPM);
        //X = SPIN UP SHOOTER



        //RIGHT BUMPER = intake; LEFT BUMPER = Intake reverse;
        g1.getGamepadButton(GamepadKeys.Button.A).whileHeld(robot.intake::intake);
        g1.getGamepadButton(GamepadKeys.Button.A).whenReleased(robot.intake::stop);



        //SHOOTER
        //DPAD_UP = power + 100; DPAD_DOWN = power - 100;
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            shooterRPM += 100;
            shooterRPM = Math.max(0, Math.min(6000, shooterRPM));
            robot.shooter.setTargetRPM(shooterRPM);
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            shooterRPM -= 100;
            shooterRPM = Math.max(0, Math.min(6000, shooterRPM));
            robot.shooter.setTargetRPM(shooterRPM);
        });

        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spinUp);
        g1.getGamepadButton(GamepadKeys.Button.X).whenReleased(robot.shooter::stop);



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
        robot.shooter.printTelemetry(pen);
        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
