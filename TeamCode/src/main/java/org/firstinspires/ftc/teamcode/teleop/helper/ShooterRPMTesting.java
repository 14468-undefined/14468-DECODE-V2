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

@TeleOp(name = "ShooterRPMTesting", group = "TeleOp")
public class ShooterRPMTesting extends SampleCommandTeleop {

    private BaseRobot robot;
    private FtcDashboard dash;
    private HardwareMap hwMap;


    private double intakePower = 0.0;
    private final double maxPower = 1.0;
    private final double minPower = -1.0;
    private final double adjustSpeed = 0.02; // how fast power changes per tick
    private final double step = 0.05;//for Y and A

    @Override
    public void onInit() {
        robot = new BaseRobot(hwMap, new Pose2d(new Vector2d(0,0), 0));






    }

    @Override
    public void onStart() {


        robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));


        //DPAD_UP = RPM + 100; DPAD_DOWN = RPM - 100;
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> robot.shooter.setTargetRPM(robot.shooter.getCurrentRPM() - 100));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> robot.shooter.setTargetRPM(robot.shooter.getCurrentRPM() + 100));

        //X = SPIN UP SHOOTER
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spinUp);


        //RIGHT BUMPER = intake; LEFT BUMPER = Intake reverse;
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(robot.intake::intakeReverse);
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(robot.intake::intake);


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
