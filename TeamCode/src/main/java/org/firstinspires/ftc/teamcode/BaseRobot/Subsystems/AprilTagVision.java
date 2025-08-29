package org.firstinspires.ftc.teamcode.BaseRobot.Subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class AprilTagVision {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // ---- constants you can tune ----
    public static final double DESIRED_DISTANCE = 12.0; // inches
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;

    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    public AprilTagVision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /** Default: drive to any detected tag */
    public PoseVelocity2d getDriveCommand() {
        return getDriveCommand(-1);
    }

    /** Drive command for RR, targeting a specific tagId (-1 = any) */
    public PoseVelocity2d getDriveCommand(int desiredId) {
        desiredTag = null;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) return null;

        // pick matching tag if desiredId >= 0
        for (AprilTagDetection tag : detections) {
            if (desiredId < 0 || tag.id == desiredId) {
                desiredTag = tag;
                break;
            }
        }
        if (desiredTag == null) return null;

        double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive  = clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn   = clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        return new PoseVelocity2d(new Vector2d(drive, strafe), turn);
    }

    /** Public getter for the last chosen tag */
    public AprilTagDetection getDesiredTag() {
        return desiredTag;
    }

    /** Public getter for desired distance */
    public double getDesiredDistance() {
        return DESIRED_DISTANCE;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }
}
