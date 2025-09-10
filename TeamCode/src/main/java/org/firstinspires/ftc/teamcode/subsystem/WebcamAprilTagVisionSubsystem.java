package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class WebcamAprilTagVisionSubsystem extends UndefinedSubsystemBase {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // PID-ish constants
    public static final double DESIRED_DISTANCE = 12.0; // inches
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;

    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    public WebcamAprilTagVisionSubsystem(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    // ---- Command-based drive to tag ----
    public CommandBase getDriveToTagCommand(DriveSubsystem drive) {
        return this.runEnd(() -> {
            // update detections
            List<AprilTagDetection> detections = aprilTag.getDetections();
            desiredTag = detections.isEmpty() ? null : detections.get(0);

            if (!hasTarget()) {
                drive.rest();
                return;
            }

            // PID-ish movement
            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            double forward  = clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe   = clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            double turn     = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            drive.driveFieldcentric(strafe, forward, turn, 0.3);

        }, drive::rest);
    }

    public boolean hasTarget() {
        return desiredTag != null;
    }

    public AprilTagDetection getDesiredTag() {
        return desiredTag;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine("Webcam Vision:");
        t.addLine("Tag Detected: " + hasTarget());
        if (hasTarget()) {
            t.addLine("Tag ID: " + desiredTag.id);
            t.addLine(String.format("X: %.2f  Y: %.2f  Range: %.2f",
                    desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.range));
        }
    }

    @Override
    public void periodic() {
        // optional: you can process detections here if you want
    }
}
