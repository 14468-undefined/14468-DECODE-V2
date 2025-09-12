package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "IntakeOuttakeTeleop", group = "TeleOp")
public class IntakeOuttakeTeleop extends SampleCommandTeleop {

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private FtcDashboard dash;
    private DriveSubsystem drive;


    private double intakePower = 0.0;
    private final double maxPower = 1.0;
    private final double minPower = -1.0;
    private final double adjustSpeed = 0.02; // how fast power changes per tick
    private final double step = 0.05;//for Y and A

    @Override
    public void onInit() {
        intake = robot.intake;
        shooter = robot.shooter;
        drive = robot.drive;

        // inside loop
        /*double rightStickY = - g1.getRightY(); // up = positive
        if (Math.abs(rightStickY) > 0.1) { // deadzone
            intakePower += rightStickY * adjustSpeed;
            intakePower = Math.max(minPower, Math.min(maxPower, intakePower));
        }

         */


        robot.drive.setDefaultCommand(new RunCommand(() -> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY(), -g1.getLeftX()), -g1.getRightX())), robot.drive));


        //trigger
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>.05).whileActiveOnce();
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>.05).whileActiveOnce();
        //g2.getGamepadButton(GamepadKeys.Button.A).whenActive(()->robot.drive.drive.resetHeadingRelative());



        //right trigger = shooter forwards
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whileActiveContinuous(new RunCommand(() -> robot.shooter.spinUp(), robot.shooter));

        //left trigger = shooter backwards
        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whileActiveContinuous(new RunCommand(() -> robot.shooter.spinUpReverse(), robot.shooter));



        // Increase intake power with Y
        /*g1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            double newPower = Math.min(maxPower, intake.getIntakePower() + step);
            intake.setIntakePower(newPower);
            pen.addLine("Intake Power increased: " + newPower);
        });


        // Decrease intake power with A
        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            double newPower = Math.min(minPower, intake.getIntakePower() - step);
            intake.setIntakePower(newPower);
            pen.addLine("Intake Power decreased: " + newPower);
        });


         */
        // Run intake forward with Right Bumper
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(intake::intake);

        // Run intake reverse with Left Bumper
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(intake::intakeReverse);




        // Stop intake when bumpers released
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(intake::stop);
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(intake::stop);

//______________________________________
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(shooter::spinUp);
        //g1.getGamepadButton(GamepadKeys.Button.B).whileHeld(shooter::spinUpReverse);

        //stop shooter when released
        g1.getGamepadButton(GamepadKeys.Button.X).whenReleased(shooter::stop);
        //g1.getGamepadButton(GamepadKeys.Button.B).whenReleased(shooter::stop);

    }

    @Override
    public void onStart() {
        // Nothing extra needed here for now
        pen.bold();
        pen.addLine("CONTROLS");
        pen.reset();
        pen.addLine("Intake:");
        pen.addLine();
        pen.addLine("Press Y to increase power");
        pen.addLine("Press A to decrease power");
        pen.addLine("Press left bumper to run reverse");
        pen.addLine("press right bumper to run forwards");
        pen.addLine();
        pen.addLine("Outtake");
        pen.addLine("Press b to run motor");
        pen.addLine("press x to run reverse");
    }

    @Override
    public void onLoop() {
        // Print intake telemetry every loop
        intake.printTelemetry(pen);
    }

    @Override
    public void onStop() {
        intake.stop();
    }
}
