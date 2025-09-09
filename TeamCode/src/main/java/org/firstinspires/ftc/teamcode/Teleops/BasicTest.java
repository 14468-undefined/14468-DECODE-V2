package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.AprilTagVision;

@TeleOp
public class BasicTest extends LinearOpMode {

    volatile BaseRobot robot;
    private volatile boolean endTeleop = false;

    // --- Add vision instance ---
    private AprilTagVision vision;

    @Override
    public void runOpMode() throws InterruptedException {

        Thread driveThread = new Thread(this::driveLoop);

        waitForStart();

        robot = new BaseRobot(hardwareMap);
        vision = new AprilTagVision(hardwareMap);  // init separate vision class

        driveThread.start();

        while (!isStopRequested() && opModeIsActive()) {


            

            telemetry.addLine("AprilTag Assist (hold Y)");
            if (vision.hasTarget()) {
                //telemetry.addData("Tag ID", vision.getTargetId());
                telemetry.addData("X", vision.getTagX());
                telemetry.addData("Y", vision.getTagY());
            } else {
                telemetry.addLine("No Tag Detected");
            }
            telemetry.update();
        }

        endTeleop = true;
        driveThread.join();
    }

    public void driveLoop() {
        while (!endTeleop) {

            double driveSpeed = 1.0;

            if (gamepad1.dpad_up) driveSpeed = 1;
            if (gamepad1.dpad_left || gamepad1.dpad_right) driveSpeed = 0.5;
            if (gamepad1.dpad_down) driveSpeed = 0.25;
            if (gamepad1.a) driveSpeed = 0.125;

            double forward = -gamepad1.left_stick_y * driveSpeed;
            double strafe  = -gamepad1.left_stick_x * driveSpeed;
            double turn    = -gamepad1.right_stick_x * driveSpeed;

            PoseVelocity2d cmd;

            if (gamepad1.y && vision.hasTarget()) {
                // --- Full AprilTag driving assist from vision class ---
                cmd = vision.getDriveCommand();
            } else {
                // --- Manual drive ---
                cmd = new PoseVelocity2d(new Vector2d(forward, strafe), turn);
            }

            robot.drive.setDrivePowers(cmd);
        }
    }
}
