package org.firstinspires.ftc.teamcode.teleop.comp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SpinUpShooterCommand;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;


@TeleOp
public class Meet1Teleop extends SampleCommandTeleop {
    @Override
    public void onInit() {


        robot.drive.setDefaultCommand(robot.drive.getDriveFieldcentric(()->g1.getLeftX(),()->g1.getLeftY(), ()->-g1.getRightX(), .75));
        //Speed Controls, slowmode * fastmode


        //trigger
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>.05).whileActiveOnce();
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>.05).whileActiveOnce();
        g1.getGamepadButton(GamepadKeys.Button.A).whenActive(()->robot.drive.drive.resetHeadingRelative());



        //slider encoder auto reseter with touch sensor
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