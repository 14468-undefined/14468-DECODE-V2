package org.firstinspires.ftc.teamcode.BaseRobot.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.Subsystems.AprilTagVision;

public class DriveToAprilTagAction implements Action {
    private final MecanumDrive drive;
    private final AprilTagVision aprilTag;
    private final int desiredTagId;  // -1 = any tag
    private final Pose2d finalPose;  // optional pose to set when tag reached

    private boolean poseUpdated = false;

    // Constructor for any tag, no final pose
    public DriveToAprilTagAction(MecanumDrive drive, AprilTagVision aprilTag) {
        this(drive, aprilTag, -1, null);
    }

    // Constructor for any tag with final pose
    public DriveToAprilTagAction(MecanumDrive drive, AprilTagVision aprilTag, double endX, double endY, double endHeading) {
        this(drive, aprilTag, -1, new Pose2d(endX, endY, endHeading));
    }

    // Constructor for specific tag ID with optional final pose
    public DriveToAprilTagAction(MecanumDrive drive, AprilTagVision aprilTag, int tagId, Pose2d finalPose) {
        this.drive = drive;
        this.aprilTag = aprilTag;
        this.desiredTagId = tagId;
        this.finalPose = finalPose;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        PoseVelocity2d command = aprilTag.getDriveCommand(desiredTagId);
        if (command != null) {
            drive.setDrivePowers(command);

            if (aprilTag.getDesiredTag() == null) return true; // keep running until a tag is detected
            double rangeError = aprilTag.getDesiredTag().ftcPose.range - aprilTag.getDesiredDistance();

            // Stop if within 1 inch
            if (Math.abs(rangeError) < 1.0) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                // Update pose estimate once if finalPose is provided
                if (finalPose != null && !poseUpdated) {
                    //drive.setPoseEstimate(finalPose);
                    poseUpdated = true;
                }

                return false; // stop action
            }

            return true; // keep running
        } else {
            // No tag detected → stop
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false;
        }
    }
}
