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
public class AprilTagHeadingAdjust extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // turning gain
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_TURN = 0.3;

    BaseRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        robot = new BaseRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double turn    = -gamepad1.right_stick_x; // default manual turn

            // --- Auto-heading assist if a tag is visible ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                // heading error from tag (bearing in degrees)
                double headingError = tag.ftcPose.bearing;

                // override turn with correction if right stick is NOT being used
                if (Math.abs(turn) < 0.05) { // small deadband
                    turn = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                }
            }

            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), turn));

            telemetry.addLine("Drive with joysticks. Release right stick to auto-face tag.");
            telemetry.update();
        }
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
