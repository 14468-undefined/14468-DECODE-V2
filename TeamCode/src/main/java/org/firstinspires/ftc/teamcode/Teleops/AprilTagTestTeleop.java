package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagTestTeleop extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double DESIRED_DISTANCE = 12.0; // inches
    private double SPEED_GAIN = 0.02;
    private double STRAFE_GAIN = 0.015;
    private double TURN_GAIN = 0.01;

    private double MAX_AUTO_SPEED = 0.5;
    private double MAX_AUTO_STRAFE = 0.5;
    private double MAX_AUTO_TURN = 0.3;

    private static final double SMOOTHING = 0.2;

    BaseRobot robot;

    // smoothing variables
    private double prevForward = 0;
    private double prevStrafe = 0;
    private double prevTurn = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize Webcam & AprilTag Processor ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // --- Initialize drive ---
        robot = new BaseRobot(hardwareMap);

        waitForStart();

        AprilTagDetection lastTag = null; // store last seen tag

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // --- Assist mode: drive to tag while holding Y ---
            if (gamepad1.y) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (!detections.isEmpty()) {
                    lastTag = detections.get(0); // update last seen tag
                }

                if (lastTag != null) {
                    double rangeError = lastTag.ftcPose.range - DESIRED_DISTANCE;
                    double headingError = lastTag.ftcPose.bearing;
                    double yawError = lastTag.ftcPose.yaw;

                    double autoForward = clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double autoStrafe = clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    double autoTurn = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    // apply smoothing
                    forward = autoForward * SMOOTHING + prevForward * (1 - SMOOTHING);
                    strafe  = autoStrafe * SMOOTHING + prevStrafe * (1 - SMOOTHING);
                    turn    = autoTurn * SMOOTHING + prevTurn * (1 - SMOOTHING);

                    prevForward = forward;
                    prevStrafe = strafe;
                    prevTurn = turn;
                }
            }

            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), turn));

            telemetry.addLine("Use Y to drive to tag");
            telemetry.update();
        }
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
