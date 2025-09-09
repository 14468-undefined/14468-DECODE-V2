package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

@TeleOp(name="HuskyLensTest", group="Test")

public class HuskyLensTest extends LinearOpMode {

    BaseRobot robot;

    boolean dismissTelemInstructions = false;

    // PD control variables
    double lastErrorX = 0;
    double kPRotate = 0.002;   // <-- tune this
    double kDRotate = 0.0002;  // <-- tune this

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BaseRobot(hardwareMap);
        robot.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        robot.huskyLens.initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (!dismissTelemInstructions) {
                telemetry.addLine("press 'a' for april tag detection");
                telemetry.addLine("press 'b' for color detection");
                telemetry.addLine("press 'x' to dismiss");
            }

            if (gamepad1.x) {
                dismissTelemInstructions = true;
            }

            // AprilTag centering
            if (gamepad1.a) {
                robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                HuskyLens.Block[] blocks = robot.huskyLens.blocks();
                HuskyLens.Block target = null;

                if (blocks != null && blocks.length > 0) {
                    target = blocks[0]; // just take first for now
                }

                if (target != null) {
                    double errorX = target.x - 320;  // 640px wide, center = 320

                    // Deadband
                    if (Math.abs(errorX) < 10) errorX = 0;

                    // Derivative
                    double derivative = errorX - lastErrorX;
                    lastErrorX = errorX;

                    // PD control
                    double turn = kPRotate * errorX + kDRotate * derivative;

                    // Drive: rotate only
                    robot.drive.setDrivePowers(
                            new PoseVelocity2d(new Vector2d(0, 0), turn)
                    );

                    telemetry.addData("Tag X", target.x);
                    telemetry.addData("ErrorX", errorX);
                    telemetry.addData("Turn", turn);
                } else {
                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    telemetry.addData("Status", "No Tag Detected");
                }
            }

            // Color detection mode
            else if (gamepad1.b) {
                robot.colorProportionalController(telemetry);
            }

            // Normal manual driving
            else {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x),
                        -gamepad1.right_stick_x));
                telemetry.addData("Status", "Manual Drive");
            }

            telemetry.update();
        }
    }
}
