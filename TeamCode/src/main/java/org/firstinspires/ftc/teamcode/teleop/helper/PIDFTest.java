package org.firstinspires.ftc.teamcode.teleop.helper;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "PIDFTest", group = "TeleOp")
public class PIDFTest extends SampleCommandTeleop {

    int shooterRPM = 0;
    boolean shooterEnabled = false;

    @Override
    public void onInit() {
        // Default: keep shooter PID running when enabled
        robot.shooter.setTargetRPM(shooterRPM);

        pen.addLine("Shooter PIDF Tuning Mode");
        pen.addLine("X = Toggle Shooter On/Off");
        pen.addLine("DPAD_UP = +100 RPM");
        pen.addLine("DPAD_DOWN = -100 RPM");
        pen.update();

        robot.shooter.setTargetRPM(shooterRPM);
    }

    @Override
    public void onStart() {

        // Toggle shooter on/off with X
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(robot.shooter::spinUp);
        g1.getGamepadButton(GamepadKeys.Button.X).whenReleased(robot.shooter::stop);

        // Adjust RPM with DPad
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            shooterRPM += 100;
            shooterRPM = Math.min(6000, shooterRPM);
            robot.shooter.setTargetRPM(shooterRPM);
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            shooterRPM -= 100;
            shooterRPM = Math.max(0, shooterRPM);
            robot.shooter.setTargetRPM(shooterRPM);
        });
    }

    @Override
    public void onLoop() {
        pen.addLine("SHOOTER TUNING");
        pen.addData("Enabled", shooterEnabled);
        pen.addData("Target RPM", shooterRPM);
        robot.shooter.printTelemetry(pen);
    }

    @Override
    public void onStop() {
        robot.shooter.stop();
    }
}
