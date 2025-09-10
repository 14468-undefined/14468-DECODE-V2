package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

@TeleOp(name = "Intake TeleOp", group = "TeleOp")
public class IntakeTeleop extends OpMode {

    private IntakeSubsystem intake;

    private FtcDashboard dash;
    @Override
    public void init() {
        // Wrap telemetry in your custom colorful telemetry
        ColorfulTelemetry cTelemetry = new ColorfulTelemetry(telemetry, dash);

        intake = new IntakeSubsystem(hardwareMap, cTelemetry);

        telemetry.addLine("Intake TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Map gamepad controls:
        // Right bumper = intake
        // Left bumper = reverse intake
        // Neither = stop
        if (gamepad1.right_bumper) {
            intake.intake();
        } else if (gamepad1.left_bumper) {
            intake.intakeReverse();
        } else {
            intake.stop();
        }

        // Update subsystem telemetry
        //intake.printTelemetry(new ColorfulTelemetry(telemetry));
    }
}
