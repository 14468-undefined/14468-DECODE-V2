package org.firstinspires.ftc.teamcode.BaseRobot.Actions;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.Subsystems.AprilTagVision;

public class TeleopDriveToAprilTagAction {

    private final MecanumDrive drive;
    private final AprilTagVision vision;

    public TeleopDriveToAprilTagAction(MecanumDrive drive, AprilTagVision vision) {
        this.drive = drive;
        this.vision = vision;
    }

    // Run this every loop when button is pressed
    public void driveToTag() {
        if (vision.hasTarget()) {
            double errorX = vision.getTagX() - 320; // adjust center pixel
            double errorY = vision.getTagY() - 240; // adjust vertical pixel if needed

            // Simple proportional control
            double kP = 0.003;
            double strafe = -errorX * kP;  // left-right correction
            double forward = -0.5;         // constant forward speed
            double turn = 0;               // you can add heading correction

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(forward, strafe),
                    turn
            ));
        } else {
            // stop if no tag
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
    }
}