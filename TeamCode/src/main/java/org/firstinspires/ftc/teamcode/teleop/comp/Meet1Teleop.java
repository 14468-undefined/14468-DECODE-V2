package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;

import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.command.SpinUpShooterCommand;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;


@TeleOp
public class Meet1Teleop extends SampleCommandTeleop {


    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        // Schedule the command


        //spinup shooter
        new Trigger(()->g2.getButton(GamepadKeys.Button.Y)).whenActive(new SpinUpShooterCommand(robot.shooter));
        new Trigger(()->g2.getButton(GamepadKeys.Button.A)).whenActive(new ShootCommand(robot, 3, 2));//shooter confirmation

        //
        //INTAKE---------------------
        robot.intake.setIntakePower(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        Trigger intakeTrigger = new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);
        intakeTrigger.whenActive(() -> robot.intake.intake());




        //DRIVE----------------------
        robot.drive.setDefaultCommand(robot.drive.getDriveFieldcentric(()->g1.getLeftX(),()->g1.getLeftY(), ()->-g1.getRightX(), .75));
        //Speed Controls, slowmode * fastmode

        g1.getGamepadButton(GamepadKeys.Button.A).whenActive(()->robot.drive.drive.resetHeadingRelative());



        //slider encoder auto reset with touch sensor
        //new Trigger(()->robot.extendo.isSliderAtRest()).whenActive(new InstantCommand(robot.extendo::resetEncoders));

    }

    @Override
    public void onStart() {

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