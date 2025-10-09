package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import java.util.Optional;

public class WebcamAprilTagVisionSubsystem extends UndefinedSubsystemBase {

    private Servo rotation;
    private VisionPortal webcam;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private final HardwareMap hardwareMap;

    // PID constants
    private static final double DESIRED_DISTANCE = 12.0; // inches
    private static final double P_DIST = 0.04, I_DIST = 0.001, D_DIST = 0.01;
    private static final double P_HEADING = 0.02, I_HEADING = 0.0005, D_HEADING = 0.005;
    private static final double P_STRAFE = 0.03, I_STRAFE = 0.001, D_STRAFE = 0.01;
    private static final double MAX_SPEED = 0.5, MAX_TURN = 0.3, MAX_STRAFE = 0.5;

    // PID state
    private double distIntegral = 0, lastDistError = 0;
    private double headingIntegral = 0, lastHeadingError = 0;
    private double strafeIntegral = 0, lastStrafeError = 0;

    public WebcamAprilTagVisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        aprilTag = new AprilTagProcessor.Builder().build();
        buildVisionPortal();

        rotation = hardwareMap.get(Servo.class, "hood");
        rotation.scaleRange(Constants.WebcamConstants.rotationMin, Constants.WebcamConstants.rotationMax);
        rotation.setPosition(Constants.WebcamConstants.rotationGoal);
    }

    private void buildVisionPortal() {
        webcam = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        desiredTag = null;
        resetPID();
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

    public boolean isAtTarget() {
        if (!hasTarget()) return false;
        double error = Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE);
        return error < 0.5; // or whatever tolerance you want
    }

    /** Command that drives the robot to the detected tag using PID */
    public CommandBase getDriveToTagCommand(DriveSubsystem drive) {
        return this.runEnd(() -> {
            if (webcam == null) {
                drive.rest();
                return;
            }

            List<AprilTagDetection> detections = aprilTag.getDetections();
            desiredTag = detections.isEmpty() ? null : detections.get(0);

            if (!hasTarget()) {
                drive.rest();
                resetPID();
                return;
            }

            double distError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double headingError = desiredTag.ftcPose.bearing;
            double strafeError = -desiredTag.ftcPose.yaw;

            // PID calculations
            double forward = pid(distError, distIntegral, lastDistError, P_DIST, I_DIST, D_DIST);
            double turn = pid(headingError, headingIntegral, lastHeadingError, P_HEADING, I_HEADING, D_HEADING);
            double strafe = pid(strafeError, strafeIntegral, lastStrafeError, P_STRAFE, I_STRAFE, D_STRAFE);

            // Clip outputs
            forward = clip(forward, -MAX_SPEED, MAX_SPEED);
            turn = clip(turn, -MAX_TURN, MAX_TURN);
            strafe = clip(strafe, -MAX_STRAFE, MAX_STRAFE);

            drive.driveFieldcentric(strafe, forward, turn, 0.5);

            // Update integrals and last errors
            distIntegral += distError;
            lastDistError = distError;

            headingIntegral += headingError;
            lastHeadingError = headingError;

            strafeIntegral += strafeError;
            lastStrafeError = strafeError;

        }, drive::rest);
    }

    private double pid(double error, double integral, double lastError, double p, double i, double d) {
        return p * error + i * integral + d * (error - lastError);
    }

    private void resetPID() {
        distIntegral = headingIntegral = strafeIntegral = 0;
        lastDistError = lastHeadingError = lastStrafeError = 0;
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
        // optional: automatic processing if needed
    }
}
