package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;


import java.util.List;

public class WebcamVisionSubsystem extends UndefinedSubsystemBase {

    /** Vision Components **/
    private Servo rotation;
    private VisionPortal webcam;
    private final HardwareMap hardwareMap;

    // Two processors: one for AprilTag, one for artifact (empty for now)
    private AprilTagProcessor aprilTagProcessor;
    private ObjectDetectionProcessor artifactProcessor; // placeholder

    /** AprilTag State **/
    private AprilTagDetection desiredTag;

    /** PID Constants **/
    private static final double DESIRED_DISTANCE = 12.0; // inches
    private static final double P_DIST = 0.04, I_DIST = 0.001, D_DIST = 0.01;
    private static final double P_HEADING = 0.02, I_HEADING = 0.0005, D_HEADING = 0.005;
    private static final double P_STRAFE = 0.03, I_STRAFE = 0.001, D_STRAFE = 0.01;
    private static final double MAX_SPEED = 0.5, MAX_TURN = 0.3, MAX_STRAFE = 0.5;

    /** PID State **/
    private double distIntegral = 0, lastDistError = 0;
    private double headingIntegral = 0, lastHeadingError = 0;
    private double strafeIntegral = 0, lastStrafeError = 0;

    /** Vision Modes **/
    public enum VisionMode {
        APRILTAG,
        ARTIFACT
    }

    private VisionMode currentMode = VisionMode.APRILTAG;

    public WebcamVisionSubsystem(HardwareMap hwMap) {
        this.hardwareMap = hwMap;

        // Initialize processors
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        artifactProcessor = new ObjectDetectionProcessor(); // currently empty

        // Build vision portal with both processors
        buildVisionPortal();

        rotation = hardwareMap.get(Servo.class, "hood");
        rotation.scaleRange(Constants.WebcamConstants.ROTATION_MIN, Constants.WebcamConstants.ROTATION_MAX);
        rotation.setPosition(Constants.WebcamConstants.ROTATION_GOAL);
    }

    /** Builds the webcam vision system */
    private void buildVisionPortal() {
        webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(artifactProcessor)
                .build();

        desiredTag = null;
        resetPID();
    }

    /** Switches between artifact and AprilTag mode */
    public void setVisionMode(VisionMode mode) {
        if (mode == currentMode) return;

        currentMode = mode;
        desiredTag = null;
        resetPID();

        switch (mode) {
            case APRILTAG:
                rotation.setPosition(Constants.WebcamConstants.ROTATION_GOAL);
                aprilTagProcessor.setActive(true);
                artifactProcessor.setActive(false);
                break;

            case ARTIFACT:
                rotation.setPosition(Constants.WebcamConstants.ROTATION_GROUND);
                aprilTagProcessor.setActive(false);
                artifactProcessor.setActive(true);
                break;
        }
    }

    /** Start the vision camera */
    public void startVision() {
        if (webcam == null) buildVisionPortal();
        desiredTag = null;
        resetPID();
    }

    /** Stop the vision camera */
    public void stopVision() {
        if (webcam != null) {
            webcam.close();
            webcam = null;
            desiredTag = null;
        }
    }

    /** Whether an AprilTag is detected */
    public boolean hasTarget() {
        return desiredTag != null;
    }

    public boolean isAtTarget() {
        if (!hasTarget()) return false;
        double error = Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE);
        return error < 0.5;
    }

    /** Command that drives to detected AprilTag using PID */
    public CommandBase getDriveToTagCommand(DriveSubsystem drive) {
        return this.runEnd(() -> {
            if (currentMode != VisionMode.APRILTAG) return; // Only run in tag mode

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            desiredTag = detections.isEmpty() ? null : detections.get(0);

            if (!hasTarget()) {
                drive.rest();
                resetPID();
                return;
            }

            double distError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double strafeError = -desiredTag.ftcPose.yaw;

            double forward = pid(distError, distIntegral, lastDistError, P_DIST, I_DIST, D_DIST);
            double turn = pid(headingError, headingIntegral, lastHeadingError, P_HEADING, I_HEADING, D_HEADING);
            double strafe = pid(strafeError, strafeIntegral, lastStrafeError, P_STRAFE, I_STRAFE, D_STRAFE);

            forward = clip(forward, -MAX_SPEED, MAX_SPEED);
            turn = clip(turn, -MAX_TURN, MAX_TURN);
            strafe = clip(strafe, -MAX_STRAFE, MAX_STRAFE);

            drive.driveFieldcentric(strafe, forward, turn, 0.5);

            distIntegral += distError;
            lastDistError = distError;
            headingIntegral += headingError;
            lastHeadingError = headingError;
            strafeIntegral += strafeError;
            lastStrafeError = strafeError;

        }, drive::rest);
    }

    //Empty processor placeholder for artifact detection
    /** Empty processor placeholder for artifact detection */
    private static class ObjectDetectionProcessor implements VisionProcessor {

        private boolean active = false;

        public void setActive(boolean active) {
            this.active = active;
        }

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
            // Called when vision starts
        }

        @Override
        public Object processFrame(android.graphics.Bitmap bitmap, long captureTimeNanos) {
            if (!active) return null;
            // TODO: Add artifact detection here later
            return null;
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {
            // Optional: draw bounding boxes or annotations
        }
    }





    /** Helper Methods **/
    private double pid(double error, double integral, double lastError, double p, double i, double d) {
        return p * error + i * integral + d * (error - lastError);
    }

    private void resetPID() {
        distIntegral = headingIntegral = strafeIntegral = 0;
        lastDistError = lastHeadingError = lastStrafeError = 0;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine("Webcam Vision:");
        t.addLine("Mode: " + currentMode);
        if (currentMode == VisionMode.APRILTAG) {
            t.addLine("Tag Detected: " + hasTarget());
            if (hasTarget()) {
                t.addLine("Tag ID: " + desiredTag.id);
                t.addLine(String.format("X: %.2f  Y: %.2f  Range: %.2f",
                        desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.range));
            }
        } else {
            t.addLine("Artifact Detection: (not implemented yet)");
        }
    }

    @Override
    public void periodic() {
        // could automatically switch based on servo position, if desired
    }
}
