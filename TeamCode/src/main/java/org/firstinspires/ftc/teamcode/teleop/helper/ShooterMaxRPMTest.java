package org.firstinspires.ftc.teamcode.teleop.helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Shooter Max RPM Test", group = "Test")
public class ShooterMaxRPMTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MotorEx shooterLeft = new MotorEx(hardwareMap, "shooterLeft");
        MotorEx shooterRight = new MotorEx(hardwareMap, "shooterRight");

        shooterRight.setInverted(false);
        shooterLeft.setInverted(true);

        final double TICKS_PER_REV = 28.0;

        telemetry.addLine("Press A to spin up, B to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Spin up or stop based on buttons ---
            if (gamepad1.a) {
                shooterLeft.set(1.0);
                shooterRight.set(1.0);
            } else if (gamepad1.b) {
                shooterLeft.set(0);
                shooterRight.set(0);
            }

            // --- Read current velocities ---
            double rawL = shooterLeft.getVelocity();   // raw ticks/sec
            double rawR = shooterRight.getVelocity();
            double avgRaw = (rawL + rawR) / 2.0;

            // Convert to RPM using ticks/rev
            double rpm = avgRaw * 60.0 / TICKS_PER_REV;

            // --- Read battery voltage ---
            double voltage = 0;
            for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                voltage = vs.getVoltage();
            }

            // --- Display telemetry ---
            telemetry.addData("rawLeft (ticks/sec)", rawL);
            telemetry.addData("rawRight (ticks/sec)", rawR);
            telemetry.addData("avgRaw", avgRaw);
            telemetry.addData("computed RPM (using 28 tpr)", rpm);
            telemetry.addData("battery voltage", "%.2f", voltage);
            telemetry.update();

            sleep(50);
        }
    }
}
