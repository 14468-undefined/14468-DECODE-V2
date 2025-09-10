package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.UndefinedSubsystemBase;

import java.util.List;
import java.util.function.DoubleSupplier;

public class WebcamAprilTagVisionSubsystem extends UndefinedSubsystemBase {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // PID gains
    private static final double DIST_KP = 0.02;
    private static final double STRAFE_KP = 0.015;
    private static final double TURN_KP = 0.01;

    private static final double MAX_DRIVE = 0.5;
    private static final double MAX_STRAFE = 0.5;
    private static final double MAX_TURN = 0.3;

    private static final double DESIRED_DISTANCE = 12.0; // inches

    public WebcamAprilTagVisionSubsystem(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /** Update detections and pick a desired tag (any if id < 0) */
    public void updateDetections(int desiredId) {
        desiredTag = null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection tag : detections) {
            if (desiredId < 0 || tag.id == desiredId) {
                desiredTag = tag;
                break;
            }
        }
    }

    /** Returns true if a tag is currently detected */
    public boolean hasTarget() {
        return desiredTag != null;
    }

    /** Returns the X/Y position of the detected tag, or NaN if none */
    public double getTagX() { return hasTarget() ? desiredTag.ftcPose.x : Double.NaN; }
    public double getTagY() { return hasTarget() ? desiredTag.ftcPose.y : Double.NaN; }
    public double getTagYaw() { return hasTarget() ? desiredTag.ftcPose.yaw : Double.NaN; }

    /** Command to drive robot toward the detected tag (PID-style proportional only) */
    public CommandBase driveToTagCommand(DoubleSupplier getForwardPower, DoubleSupplier getStrafePower, DoubleSupplier getTurnPower) {
        return this.runEnd(() -> {
            if (desiredTag == null) return;

            double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            double forward = clip(rangeError * DIST_KP, -MAX_DRIVE, MAX_DRIVE);
            double strafe  = clip(-yawError * STRAFE_KP, -MAX_STRAFE, MAX_STRAFE);
            double turn    = clip(headingError * TURN_KP, -MAX_TURN, MAX_TURN);

            // Set values via provided DoubleSuppliers (these should call your drive subsystem)
            getForwardPower.getAsDouble(); // placeholder for drive subsystem interface
            getStrafePower.getAsDouble();  // placeholder
            getTurnPower.getAsDouble();    // placeholder

        }, () -> {
            // stop movement when command ends
            getForwardPower.getAsDouble(); // 0
            getStrafePower.getAsDouble();  // 0
            getTurnPower.getAsDouble();    // 0
        });
    }

    /** Helper clip method */
    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /** Close vision portal when done */
    public void close() {
        if (visionPortal != null) visionPortal.close();
    }


    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine("AprilTagVision Status");
        if (desiredTag != null) {
            t.addLine("Detected Tag ID: " + desiredTag.id);
            t.addLine(String.format("X: %.2f, Y: %.2f, Range: %.2f",
                    desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.range));
        } else {
            t.addLine("No tag detected");
        }
    }

    @Override
    public void periodic() {
        // optionally update detections automatically each loop
        updateDetections(-1);
    }
}
