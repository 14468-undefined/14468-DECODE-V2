package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagHeadingAdjust extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // turning assist parameters
    private static final double TURN_GAIN = 0.02;
    private static final double MAX_AUTO_TURN = 0.6;

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

            // --- reset heading when A is pressed ---
            if (gamepad1.a) {
                robot.drive.resetHeading();
            }

            // raw joystick input
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;   // non-inverted
            double turn    = -gamepad1.right_stick_x;

            // check for tags
            List<AprilTagDetection> detections = aprilTag.getDetections();
            double speedFactor = 1.0; // normal speed

            if (!detections.isEmpty()) {
                // slow down movement when tag is seen
                speedFactor = 0.3;

                // heading correction
                AprilTagDetection tag = detections.get(0);
                double headingError = tag.ftcPose.bearing;

                if (Math.abs(turn) < 0.05) { // deadband for manual stick
                    turn = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                }
            }

            // --- field-centric drive ---
            robot.drive.driveFieldCentric(strafe, forward, turn, speedFactor, telemetry);

            telemetry.addLine("Drive with joysticks.");
            telemetry.addLine("Auto-face tag when visible (release right stick).");
            telemetry.addLine("Reset heading with A.");
            telemetry.addLine("Speed factor: " + speedFactor);
            telemetry.update();
        }
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
