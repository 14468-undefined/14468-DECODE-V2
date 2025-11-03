package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.List;

public class WebcamVisionSubsystem extends UndefinedSubsystemBase {

   // private Servo rotation;
    private VisionPortal webcam;
    private final HardwareMap hardwareMap;

    private AprilTagProcessor aprilTagProcessor;
    private ObjectDetectionProcessor artifactProcessor;

    private AprilTagDetection desiredTag;

    private static final double DESIRED_DISTANCE = 12.0;
    private static final double P_DIST = 0.04, I_DIST = 0.001, D_DIST = 0.01;
    private static final double P_HEADING = 0.02, I_HEADING = 0.0005, D_HEADING = 0.005;
    private static final double P_STRAFE = 0.03, I_STRAFE = 0.001, D_STRAFE = 0.01;
    private static final double MAX_SPEED = 0.5, MAX_TURN = 0.3, MAX_STRAFE = 0.5;

    private double distIntegral = 0, lastDistError = 0;
    private double headingIntegral = 0, lastHeadingError = 0;
    private double strafeIntegral = 0, lastStrafeError = 0;

    public enum VisionMode {
        APRILTAG,
        ARTIFACT
    }

    private VisionMode currentMode = VisionMode.APRILTAG;

    public WebcamVisionSubsystem(HardwareMap hwMap) {
        this.hardwareMap = hwMap;

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        artifactProcessor = new ObjectDetectionProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                return null;
            }
        };

        buildVisionPortal();

        /*rotation = hardwareMap.get(Servo.class, "hood");
        rotation.scaleRange(Constants.WebcamConstants.ROTATION_MIN, Constants.WebcamConstants.ROTATION_MAX);
        rotation.setPosition(Constants.WebcamConstants.ROTATION_GOAL);

         */
    }

    private void buildVisionPortal() {
        webcam = new VisionPortal.Builder()
                //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(artifactProcessor)
                .build();

        desiredTag = null;
        resetPID();
    }

    public void setVisionMode(VisionMode mode) {
        if (mode == currentMode) return;

        currentMode = mode;
        desiredTag = null;
        resetPID();

        switch (mode) {
            case APRILTAG:
                //rotation.setPosition(Constants.WebcamConstants.ROTATION_GOAL);
                webcam.setProcessorEnabled(aprilTagProcessor, true);
                webcam.setProcessorEnabled(artifactProcessor, false);
                break;

            case ARTIFACT:
                //rotation.setPosition(Constants.WebcamConstants.ROTATION_GROUND);
                webcam.setProcessorEnabled(aprilTagProcessor, false);
                webcam.setProcessorEnabled(artifactProcessor, true);
                break;
        }
    }

    public void startVision() {
        if (webcam == null) buildVisionPortal();
        desiredTag = null;
        resetPID();
    }

    public void stopVision() {
        if (webcam != null) {
            webcam.close();
            webcam = null;
            desiredTag = null;
        }
    }

    public boolean hasTarget() {
        return desiredTag != null;
    }

    public boolean isAtTarget() {
        if (!hasTarget()) return false;
        double error = Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE);
        return error < 0.5;
    }

    public CommandBase getDriveToTagCommand(DriveSubsystem drive) {
        return this.runEnd(() -> {
            if (currentMode != VisionMode.APRILTAG) return;

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

    private static abstract class ObjectDetectionProcessor implements VisionProcessor {

        private boolean active = false;

        public void setEnabled(boolean enabled) {
            this.active = enabled;
        }


        public void init(int width, int height, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {}


        public Object processFrame(android.graphics.Bitmap bitmap, long captureTimeNanos) {
            if (!active) return null;
            return null;
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {}
    }

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
    public void periodic() {}
}
