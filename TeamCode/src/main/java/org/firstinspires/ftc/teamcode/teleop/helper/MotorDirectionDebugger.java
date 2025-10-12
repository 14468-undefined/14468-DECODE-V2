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

    private BaseRobot robot;
    private FtcDashboard dash;
    private HardwareMap hwMap;



    @Override
    public void onInit() {
        robot = new BaseRobot(hwMap, new Pose2d(new Vector2d(0,0), 0));






    }

    @Override
    public void onStart() {


        robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));


        robot.shooter.setTargetRPM(60);
        //X = SPIN UP SHOOTER
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spinUpRight);
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spinUpLeft);


        //RIGHT BUMPER = intake; LEFT BUMPER = Intake reverse;
        g1.getGamepadButton(GamepadKeys.Button.A).whileHeld(robot.intake::intake);



        pen.addLine("CONTROLS");
        pen.addLine();
        pen.addLine("Intake: A");
        pen.addLine("Shooter Right: X");
        pen.addLine("Shooter Left: B");
    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop
        robot.shooter.printTelemetry(pen);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
